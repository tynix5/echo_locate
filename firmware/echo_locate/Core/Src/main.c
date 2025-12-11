/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "main.h"
#include "uart2.h"
#include "dsp/filtering_functions.h"
#include "dsp/statistics_functions.h"
#include <stdlib.h>
#include <stdio.h>

#define N_TAPS				48
#define N_BLOCK				1440							// total samples from ADC before switching DMA targets
#define N_SAMPLE			(BLOCK_SIZE / 3)				// samples per microphone before switching DMA targets

#define SAMPLES_BEFORE_DET	10								// # of samples used before peak detection in cross correlation
#define SAMPLES_AFTER_DET	189								// # of samples needed after first peak detection to compute cross correlation

#define WINDOW_SIZE			2
#define N_BUFFER			WINDOW_SIZE * SAMPLE_SIZE		// buffer for previous microphone samples

#define ENERGY_THRESH		8000							// detected event energy threshold
#define UPDATE_PERIOD		0.025							// delay between position updates
#define DETECTION_TIMEOUT	CORR_IN_SIZE * SAMPLE_PERIOD	// maximum allowable delay between consecutive peak detections before timeout
#define MAX_TDOA			DETECTION_TIMEOUT

#define SAMPLE_FREQ			40000.0f						// ADC group sample rate
#define SAMPLE_PERIOD		(1.0f / SAMPLE_FREQ)
#define TIME_FREQ			100000.0f						// global timer frequency
#define TIME_PERIOD			(1.0f / TIME_FREQ)

#define SPEED_OF_SOUND		343.0f							// m/s

#define PROC_TIME			SAMPLE_SIZE * SAMPLE_PERIOD

#define N_MICS				3

// distances in meters
#define MIC0_XPOS			0
#define MIC0_YPOS			0
#define MIC1_XPOS			0
#define MIC1_YPOS			1
#define MIC2_XPOS			1
#define MIC2_YPOS			1

#if CORR_IN_SIZE > N_BUFFER
#error	"Correlation input length cannot exceed buffer size"
#endif


/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 40000 Hz

fixed point precision: 16 bits

* 0 Hz - 200 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 1500 Hz - 5000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 6000 Hz - 20000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

q15_t const ftaps_q15[N_TAPS] = {
		427, 682, 874, 755, 246, -527,
		-1261, -1613, -1405, -775, -149, -25,
		-669, -1885, -3025, -3280, -2107, 416,
		3530, 6090, 7078, 6090, 3530, 416,
		-2107 -3280, -3025, -1885, -669, -25,
		-149, -775, -1405, -1613, -1261, -527,
		246, 755, 874, 682, 427, 0
};
*/

q15_t const lowpass_taps[N_TAPS] = {
		0
};

uint16_t stream0[N_BLOCK], stream1[N_BLOCK];										// raw data streams from DMA buffers

struct MicStruct {

	arm_fir_instance_q15 hfir;								// FIR handle
	q15_t state[N_TAPS + N_SAMPLE - 1];						// lowpass FIR filter state
	q15_t samples[N_SAMPLE];								// bandpassed samples received from FPGA
	q15_t envelope[N_SAMPLE];								// scaled, lowpassed, and square rooted samples
	q15_t buffer[N_BUFFER];									// running window

	q15_t env_peak;						// envelope peak
	uint32_t env_peak_ind;				// envelope peak index
};

uint8_t dma_tgt = 0;				// M0AR written to first


/* Configure system clock for 84 MHz */
void sysclock_init(void);
/* Configure ADC1 to trigger via TIM2, and sample normal group ADC0, ADC1, and ADC2, streaming to DMA2 in double-buffer mode */
void adc1_dma_init(void);
/* 100 kHz global timer */
void tim5_time_init(void);
/* Split DMA stream into separate microphone streams, converting from biased uint16_t to q15_t */
void stream_splice(struct MicStruct * mics);
/* Compute x and y coordinates of event using trilateration */
uint8_t compute_event_pos(float * x, float * y, float mic0_x, float mic0_y,
					   float mic1_x, float mic1_y, float mic2_x, float mic2_y,
					   float mic1_delay, float mic2_delay);
/* compute_event_pos helper function */
float clamp(float in, float abs_max);
/* Envelope detector function */
void compute_envelope(struct MicStruct * mics);
/* Threshold-search function, returns index of first value in Src > thresh */
int32_t thresh_search(q15_t * src, uint32_t len, q15_t thresh);
/* simple sorting function */
void simple_sort3(int32_t * src, int32_t * dst);

int main(void)
{

	sysclock_init();

	struct MicStruct mics[N_MICS];

	// initiate lowpass FIRs
	for (uint32_t i = 0; i < N_MICS; i++)
		arm_fir_init_q15(&mics[i].hfir, N_TAPS, lowpass_taps, &mics[i].state, N_SAMPLE);

	uart2_set_fcpu(84000000);
	uart2_dma1_init(115200, USART_DATA_8, USART_STOP_1);

	tim5_time_init();

//	adc1_dma_init();

	/*
	GPIOA->MODER &= ~GPIO_MODER_MODER10;
	GPIOA->MODER |= GPIO_MODER_MODER10_0;

	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	*/

	uint32_t prev_ticks = 0, ticks = 0;

	uint8_t window_ind = 0;

	float last_pos_update = 0;								// last time position was updated
	uint8_t triggered = 0;									// has a microphone detected an event?
	uint32_t ref_sample = 0;
	uint32_t samples = 0;

	while (1)
	{

//		GPIOA->ODR |= GPIO_ODR_OD10;

		while (!!(DMA2_Stream0->CR & DMA_SxCR_CT) == dma_tgt);		// wait for stream to complete
		prev_ticks = ticks;
		ticks = TIM5->CNT;
		samples += N_SAMPLE;
//		GPIOA->ODR &= ~GPIO_ODR_OD10;
		dma_tgt = !dma_tgt;											// switch DMA targets
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;			// clear transfer complete and half complete flag

		stream_splice(&mics);

//		GPIOA->ODR |= GPIO_ODR_OD8;

		// square, lowpass, sqrt
		compute_envelope(&mics);

		// reference time is based on # ticks since last DMA buffer started
		float ref_time = prev_ticks * TIME_PERIOD;

		if (!triggered && ref_time - last_pos_update > UPDATE_PERIOD)			// wait for event detection and UPDATE_PERIOD time between updates
		{

			// select a reference sample by finding the first peak above a threshold

			// find first peak above ENERGY_THRESH in envelope for each microphone
			// if no such peak exists, place -1 at that index
			int32_t thresh_ind[N_MICS];
			for (uint32_t i = 0; i < N_MICS; i++)
				thresh_ind[i] = thresh_search(&mics[i].envelope, N_SAMPLE, ENERGY_THRESH);

			int32_t thresh_ind_sorted[N_MICS];
			simple_sort3(thresh_ind, thresh_ind_sorted);		// sort the indices of the first peaks

			// find first non -1 index, that index becomes reference sample
			for (uint32_t i = 0; i < N_MICS; i++)
			{
				if (thresh_ind_sorted[i] != -1)
				{
					ref_sample = thresh_ind_sorted[i];			// may need to adjust this for window size
					triggered = 1;
					break;
				}
			}
		}
		// sample > ref_sample + SAMPLES_AFTER_DET is wrong
		else if (triggered && samples > ref_sample + SAMPLES_AFTER_DET)		// wait until enough samples have been taken after peak is detected
		{
			// compute GCC-PHAT
			// estimate location

			// reset event detection
			triggered = 0;
			last_pos_update = TIM5->CNT * TIME_PERIOD;
		}

		if (++window_ind == WINDOW_SIZE)
			window_ind = 0;

//		GPIOA->ODR &= ~GPIO_ODR_OD8;

	}
}

void sysclock_init(void)
{
	// 16 MHz HSI oscillator is default on reset, but select anyways
	RCC->CR |= RCC_CR_HSION;
	// wait for HSI to be ready
	while (!((RCC->CR) & RCC_CR_HSIRDY));

	// enable power interface clock for APB1
	RCC->APB1ENR = RCC_APB1ENR_PWREN;

	// configure VCO to scale 2 per CubeMX
	PWR->CR |= PWR_CR_VOS_1;
	PWR->CR &= ~PWR_CR_VOS_0;

	// configure FLASH
	// instruction cache, prefetch enable, and data cache enabled
	uint32_t flash;
	flash = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN;
	flash |= 2;		// 2 wait states for flash
	FLASH->ACR = flash;

	// configure bus prescalers
	uint32_t cfgr = 0;
	cfgr &= ~RCC_CFGR_PPRE2_2;		// APB2 prescaler of 1 (84MHz)
	cfgr |= RCC_CFGR_PPRE1_2;		// APB1 prescaler of 2 (42MHZ)
	cfgr &= ~RCC_CFGR_HPRE;			// AHB prescaler of 1 (84MHz)
	RCC->CFGR = cfgr;

	// configure main PLL
	uint32_t pll_cfg = RCC->PLLCFGR;
	pll_cfg &= ~RCC_PLLCFGR_PLLQ;
	pll_cfg |= RCC_PLLCFGR_PLLQ_2; // configure Q prescaler for USB, SDIO, RNG clocks (/4)

	pll_cfg &= ~RCC_PLLCFGR_PLLP;	// main PLL division factor of 2

	pll_cfg &= ~RCC_PLLCFGR_PLLN;
	pll_cfg |= 168UL << 6;	// pll multiplication factor for VCO (x168)

	pll_cfg &= ~RCC_PLLCFGR_PLLM;
	pll_cfg |= 16UL << 0;	// pll division factor for main PLL and audio PLL (/16)

	RCC->PLLCFGR = pll_cfg;

	// enable PLL and wait for ready
	RCC->CR |= RCC_CR_PLLON;
	while (!((RCC->CR) & RCC_CR_PLLRDY));

	// select clock source
	cfgr = RCC->CFGR;
	cfgr |= RCC_CFGR_SW_1;		// select PLL as system clock
	cfgr &= ~RCC_CFGR_SW_0;
	RCC->CFGR = cfgr;

	// wait for PLL clock source to become active
	while (!((RCC->CFGR) & RCC_CFGR_SWS_1));
}

void adc1_dma_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;				// enable ADC1 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			// enable GPIOA clock

	// PA0 as analog input
	GPIOA->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1;
	// PA1 as analog input
	GPIOA->MODER |= GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1;
	// PA4 as analog input
	GPIOA->MODER |= GPIO_MODER_MODER4_0 | GPIO_MODER_MODER4_1;

	ADC1->CR2 &= ~ADC_CR2_ADON;						// turn off ADC to configure

	// APB2 clock (84 MHz) / 4 = 21 MHz
	// MAX ADC clock freq is 36 MHz (pg 106 datasheet)
	ADC->CCR &= ~ADC_CCR_ADCPRE;
	ADC->CCR |= ADC_CCR_ADCPRE_0;

	// trigger detection on rising edge
	ADC1->CR2 |= ADC_CR2_EXTEN_0;
	ADC1->CR2 &= ~ADC_CR2_EXTEN_1;

	// TIM2 TRGO event
	ADC1->CR2 |= ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;
	ADC1->CR2 &= ~(ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_3);

	// Scan mode to convert all 3 channels
	ADC1->CR1 |= ADC_CR1_SCAN;

	// EOC bit set at end of each sequence of regular conversions
	ADC1->CR2 &= ~ADC_CR2_EOCS;

	// 3 cycles before sample
	ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2);

	// 3 conversions per sequence
	ADC1->SQR1 &= ~ADC_SQR1_L;
	ADC1->SQR1 |= ADC_SQR1_L_1;

	// channel 0 (PA0 --> ADC1_IN0) is first conversion (microphone 0)
	ADC1->SQR3 &= ~ADC_SQR3_SQ1;

	// channel 1 (PA1 --> ADC1_IN1) is second conversion (microphone 1)
	ADC1->SQR3 &= ~ADC_SQR3_SQ2;
	ADC1->SQR3 |= ADC_SQR3_SQ2_0;

	// channel 4 (PA4 --> ADC1_IN4) is third conversion (microphone 2)
	ADC1->SQR3 &= ~ADC_SQR3_SQ3;
	ADC1->SQR3 |= ADC_SQR3_SQ3_2;


	/* DMA2 Channel 0, Stream 0 --> ADC1 */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;		// enable DMA2 clock

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;		// disable stream
	while (DMA2_Stream0->CR & DMA_SxCR_EN);	// wait for stream to disable

	DMA2_Stream0->PAR = (uint32_t)&(ADC1->DR);// peripheral address
	DMA2_Stream0->M0AR = (uint32_t)stream0;	// destination memory address (CT = 0)
	DMA2_Stream0->M1AR = (uint32_t)stream1;	// destination memory address (CT = 1)
	DMA2_Stream0->NDTR = N_BLOCK;			// number of units to be transmitted

	DMA2_Stream0->CR &= ~DMA_SxCR_CHSEL;	// channel 0 selected

	DMA2_Stream0->CR &= ~DMA_SxCR_PFCTRL;	// DMA is the flow controller

	// priority level: medium
	DMA2_Stream0->CR |= DMA_SxCR_PL_0;
	DMA2_Stream0->CR &= ~DMA_SxCR_PL_1;

	// disable direct mode
	// FIFO threshold of 1/2 before transfer
	DMA2_Stream0->FCR |= DMA_SxFCR_DMDIS;
	DMA2_Stream0->FCR |= DMA_SxFCR_FTH_0;
	DMA2_Stream0->FCR &= ~DMA_SxFCR_FTH_1;

	// memory data size: half word (16 bits)
	DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0;
	DMA2_Stream0->CR &= ~DMA_SxCR_MSIZE_1;

	// peripheral data size: half word (16 bits)
	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0;
	DMA2_Stream0->CR &= ~DMA_SxCR_PSIZE_1;

	// increment memory address after every transfer
	DMA2_Stream0->CR |= DMA_SxCR_MINC;

	// disable peripheral address increment
	DMA2_Stream0->CR &= ~DMA_SxCR_PINC;

	// peripheral to memory data direction
	DMA2_Stream0->CR &= ~DMA_SxCR_DIR;

	// circular mode --> reload NDTR after every transfer complete
	DMA2_Stream0->CR |= DMA_SxCR_CIRC;

	// double buffer mode; current target is memory 0 (DMA_SxM0AR)
	DMA2_Stream0->CR |= DMA_SxCR_DBM;
	DMA2_Stream0->CR &= ~DMA_SxCR_CT;


	DMA2_Stream0->CR |= DMA_SxCR_EN;		// enable DMA stream

	ADC1->CR2 |= ADC_CR2_DMA;				// enable DMA
	ADC1->CR2 |= ADC_CR2_DDS;				// DMA requests issued as long as DMA=1
	ADC1->CR2 |= ADC_CR2_ADON;				// turn on ADC
}

void tim5_time_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;				// enable TIM2 clock

	TIM5->CR1 &= ~TIM_CR1_DIR;			// upcounting
	TIM5->PSC = 839;					// 100kHz frequency
	TIM5->ARR = 0xffffffff;

	TIM5->EGR |= TIM_EGR_UG;			// generate update event

	TIM5->CR1 |= TIM_CR1_CEN;			// enable counter
}

void stream_splice(struct MicStruct * mics)
{
	for (uint32_t i = 0; i < BLOCK_SIZE; i += 3)
	{
		uint32_t ind = i / 3;

		// input stream looks like this
		// [][[]


		// DMA is targeting M1AR ---> read from M0AR memory
		// TO DO: check ranges of mic0
		if (dma_tgt)
		{
			// convert from uint16_t in range [0, 4095] from ADC to q15_t range [-1, 1]
			/*
			mic0_samp[ind] = ((int16_t)stream0[i] - 2048) << 4;
			mic1_samp[ind] = ((int16_t)stream0[i+1] - 2048) << 4;
			mic2_samp[ind] = ((int16_t)stream0[i+2] - 2048) << 4;
			*/
			for (uint32_t j = 0; j < N_MICS; j++)
				mics[j]->samples[ind] = stream0[i + j];

		}
		else
		{
			for (uint32_t j = 0; j < N_MICS; j++)
				mics[j]->samples[ind] = stream1[i + j];
		}
	}
}

uint8_t compute_event_pos(float * x, float * y, float mic0_x, float mic0_y,
					   float mic1_x, float mic1_y, float mic2_x, float mic2_y,
					   float mic1_delay, float mic2_delay)
{

	const float max_step = 0.2f;			// maximum dx/dy change per iteration in meters

	// distances from mic1 and mic2 to mic0
	const float d10 = SPEED_OF_SOUND * mic1_delay;
	const float d20 = SPEED_OF_SOUND * mic2_delay;

	float lambda = 1e-3f;

	float old_res1 = 0, old_res2 = 0;

	for (uint8_t i = 0; i < 50; i++)
	{

		// compute radii of guesses
		float r0 = sqrtf(powf(*x - mic0_x, 2) + powf(*y - mic0_y, 2));
		float r1 = sqrtf(powf(*x - mic1_x, 2) + powf(*y - mic1_y, 2));
		float r2 = sqrtf(powf(*x - mic2_x, 2) + powf(*y - mic2_y, 2));

		if (r0 == 0 || r1 == 0 || r2 == 0) return 1;

		// compute residuals (error)
		// [f]
		float res1 = r1 - r0 - d10;
		float res2 = r2 - r0 - d20;

		// compute cost, exit if small
		float cost = res1 * res1 + res2 * res2;
		float old_cost = old_res1 * old_res1 + old_res2 * old_res2;
		if (cost < 1e-8f)	return 1;

		// else, compare new residuals to ones calculated previous iteration
		if (cost < old_cost)		lambda *= 0.3f;			// reward improvement
		else						lambda *= 5.0f;			// punish bad step

		// create Jacobian
		float j11 = (*x - mic1_x) / r1 - (*x - mic0_x) / r0;
		float j12 = (*y - mic1_y) / r1 - (*y - mic0_y) / r0;
		float j21 = (*x - mic2_x) / r2 - (*x - mic0_x) / r0;
		float j22 = (*y - mic2_y) / r2 - (*y - mic0_y) / r0;

		// ([J]^T)[J] with damping on diagonal
		float prod11 = j11 * j11 + j21 * j21 + lambda;
		float prod12 = j11 * j12 + j21 * j22;
		float prod21 = prod12;
		float prod22 = j12 * j12 + j22 * j22 + lambda;

		// ([J]^T)[f]
		float g1 = j11 * res1 + j21 * res2;
		float g2 = j12 * res1 + j22 * res2;

		// solve system
		// ([J]^T)[J]delta = -([J]^T)[f]
		// delta = inv(([J]^T)[J]) * (-([J]^T)[f])
		float det = prod11 * prod22 - prod12 * prod21;

		// ill-conditioned, increase lambda and try again
		if (fabsf(det) < 1e-12f)
		{
			lambda *= 10.0f;
			continue;
		}

		float dx = (-prod22 * g1 + prod12 * g2) / det;
		float dy = (prod21 * g1 - prod11 * g2) / det;

		dx = clamp(dx, max_step);
		dy = clamp(dy, max_step);

		*x += dx;
		*y += dy;

		old_res1 = res1;
		old_res2 = res2;

		if (sqrtf(dx * dx + dy * dy) < 1e-5) return 1;
	}

	return 0;			// error if didn't converge
}

float clamp(float in, float abs_max)
{
	// clamp in between [-abs_max, abs_max]
	if (in > abs_max)
		return abs_max;
	else if (in < -abs_max)
		return -abs_max;
	else
		return in;
}

void compute_envelope(struct MicStruct * mics)
{
	// square signals and multiply with gain of 2
	for (uint32_t i = 0; i < N_MICS; i++)
		for (uint32_t j = 0; j < N_SAMPLE; j++)
			mics[i]->envelope[j] = mics[i]->samples[j] * mics[i]->samples[j] * 2;

	// Lowpass filter to remove high frequencies accumulated during scaling
	for (uint32_t i = 0; i < N_MICS; i++)
		arm_fir_fast_q15(mics[i].hfir, mics[i].envelope, mics[i].envelope, N_SAMPLE);

	// square root output
	for (uint32_t i = 0; i < N_MICS; i++)
		for (uint32_t j = 0; j < N_SAMPLE; j++)
			arm_sqrt_q15(mics[i].envelope[j], &mics[i].envelope[j]);
}

int32_t thresh_search(q15_t * src, uint32_t len, q15_t thresh)
{
	for (uint32_t i = 0; i < len; i++)
	{
		if (src[i] > thresh)
			return i;
	}

	return -1;
}

void simple_sort3(int32_t * src, int32_t * dst)
{

	if (src[0] <= src[1] && src[0] <= src[2])
	{
		dst[0] = src[0];

		if (src[1] <= src[2])
		{
			dst[1] = src[1];
			dst[2] = src[2];
		}
		else
		{
			dst[1] = src[2];
			dst[2] = src[1];
		}
	}
	else if (src[1] <= src[0] && src[1] <= src[2])
	{
		dst[0] = src[1];

		if (src[0] <= src[2])
		{
			dst[1] = src[0];
			dst[2] = src[2];
		}
		else
		{
			dst[1] = src[2];
			dst[2] = src[0];
		}
	}
	else
	{
		dst[0] = src[2];

		if (src[0] <= src[1])
		{
			dst[1] = src[0];
			dst[2] = src[1];
		}
		else
		{
			dst[1] = src[1];
			dst[2] = src[0];
		}
	}
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
