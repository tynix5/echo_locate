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
#include <stdlib.h>

#define NUM_FILTER_TAPS		46
#define BLOCK_SIZE			144
#define SAMPLE_SIZE			BLOCK_SIZE / 3
#define DECIMATION_M		4
#define DECIMATION_SIZE		SAMPLE_SIZE / DECIMATION_M

#define THRESH_EVENT		2000
#define TIMEOUT_S			1
#define EVENT_DB_TIME		0.1					// wait 100ms between updates

#define SAMPLE_FREQ			40000
#define SAMPLE_PERIOD		1.0 / SAMPLE_FREQ
#define TIME_FREQ			100000
#define TIME_PERIOD			1.0 / TIME_FREQ

// account for temperature based changes later
#define SPEED_OF_SOUND		343

// distances in meters
#define MIC0_XPOS			0
#define MIC0_YPOS			0
#define MIC1_XPOS			0
#define MIC1_YPOS			1
#define MIC2_XPOS			1
#define MIC2_YPOS			1

// mic0 is the reference mic


/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 10000 Hz

fixed point precision: 16 bits

* 0 Hz - 750 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 1000 Hz - 4500 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 4750 Hz - 5000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/


q15_t const ftaps_q15[NUM_FILTER_TAPS] = {
		  -1182, 958, 426, -1481, 1444, 514, 1091,
		  -1129, -937, -1266, -422, 518, 215, 1791,
		  23, 2421, -1198, 1486, -4132, -982, -7361,
		  -2971, 24064, -2971, -7361, -982, -4132, 1486, -1198,
		  2421, 23, 1791, 215, 518, -422, -1266, -937,
		  -1129, 1091, 514, 1444, -1481, 426, 958, -1182, 0
};

arm_fir_decimate_instance_q15 hfir0;
arm_fir_decimate_instance_q15 hfir1;
arm_fir_decimate_instance_q15 hfir2;

uint16_t stream0[BLOCK_SIZE];
uint16_t stream1[BLOCK_SIZE];
q15_t mic0_samp[SAMPLE_SIZE];
q15_t mic1_samp[SAMPLE_SIZE];
q15_t mic2_samp[SAMPLE_SIZE];

uint8_t dma_tgt = 0;				// M0AR written to first

uint8_t mic_detected_event[3] = {0};

void sysclock_config(void);
void adc1_dma_config(void);
void tim2_trig_config(void);
void tim5_time_config(void);
void stream_splice(void);
void find_filter_peaks(q15_t * mic0_filtered, q15_t * mic1_filtered, q15_t * mic2_filtered,
						uint16_t * mic0_ind, uint16_t * mic1_ind, uint16_t * mic2_ind);
void compute_event_pos(float * x, float * y, float mic0_x, float mic0_y,
					   float mic1_x, float mic1_y, float mic2_x, float mic2_y,
					   float mic1_delay, float mic2_delay);

int main(void)
{

	sysclock_config();

	q15_t mic0_state[NUM_FILTER_TAPS + SAMPLE_SIZE - 1];
	q15_t mic1_state[NUM_FILTER_TAPS + SAMPLE_SIZE - 1];
	q15_t mic2_state[NUM_FILTER_TAPS + SAMPLE_SIZE - 1];

	arm_fir_decimate_init_q15(&hfir0, NUM_FILTER_TAPS, 4, ftaps_q15, mic0_state, SAMPLE_SIZE);
	arm_fir_decimate_init_q15(&hfir1, NUM_FILTER_TAPS, 4, ftaps_q15, mic1_state, SAMPLE_SIZE);
	arm_fir_decimate_init_q15(&hfir2, NUM_FILTER_TAPS, 4, ftaps_q15, mic2_state, SAMPLE_SIZE);

	q15_t mic0_filtered[DECIMATION_SIZE];
	q15_t mic1_filtered[DECIMATION_SIZE];
	q15_t mic2_filtered[DECIMATION_SIZE];

	uart2_set_fcpu(84000000);
	uart2_dma1_config(115200, USART_DATA_8, USART_STOP_1);

	tim5_time_config();

	tim2_trig_config();
	adc1_dma_config();

//	GPIOA->MODER &= ~GPIO_MODER_MODER10;
//	GPIOA->MODER |= GPIO_MODER_MODER10_0;
//
//	GPIOA->MODER &= ~GPIO_MODER_MODER8;
//	GPIOA->MODER |= GPIO_MODER_MODER8_0;

	float mic0_timestamp = 0, mic1_timestamp = 0, mic2_timestamp = 0;
	float last_event_timestamp = 0;

	uint8_t detection_cnt = 0;						// number of microphones that have detected an event
	uint32_t sample_cnt = 0;						// # of samples collected

	uint32_t prev_cnt = 0, this_cnt = 0;

	while (1)
	{

//		GPIOA->ODR |= GPIO_ODR_OD10;
		while (!!(DMA2_Stream0->CR & DMA_SxCR_CT) == dma_tgt);		// wait for stream to complete
		prev_cnt = this_cnt;
		this_cnt = TIM5->CNT;
//		GPIOA->ODR &= ~GPIO_ODR_OD10;
		dma_tgt = !dma_tgt;								// switch DMA targets
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;			// clear transfer complete and half complete flag

		stream_splice();

//		GPIOA->ODR |= GPIO_ODR_OD8;

		// Cross correlation instead of filtering?
		arm_fir_decimate_fast_q15(&hfir0, mic0_samp, mic0_filtered, SAMPLE_SIZE);
		arm_fir_decimate_fast_q15(&hfir1, mic1_samp, mic1_filtered, SAMPLE_SIZE);
		arm_fir_decimate_fast_q15(&hfir2, mic2_samp, mic2_filtered, SAMPLE_SIZE);

		uint16_t mic0_max, mic1_max, mic2_max;			// hold indices of max values in filtered sample (0-DECIMATION_SIZE)
		find_filter_peaks(mic0_filtered, mic1_filtered, mic2_filtered, &mic0_max, &mic1_max, &mic2_max);

		float sample_based_time = sample_cnt * SAMPLE_PERIOD;
		// if a new event has been detected, update time stamps and event count
		if ((abs(mic0_filtered[mic0_max]) > THRESH_EVENT) && (sample_based_time - mic0_timestamp > EVENT_DB_TIME))
		{
			mic0_timestamp = prev_cnt * TIME_PERIOD + mic0_max * SAMPLE_PERIOD * DECIMATION_M;
			mic_detected_event[0] = 1;
			detection_cnt++;
			last_event_timestamp = mic0_timestamp;
		}

		if ((abs(mic1_filtered[mic1_max]) > THRESH_EVENT) && (sample_based_time - mic1_timestamp > EVENT_DB_TIME))
		{
			// account for sample delay later
			mic1_timestamp = prev_cnt * TIME_PERIOD + mic1_max * SAMPLE_PERIOD * DECIMATION_M;
			mic_detected_event[1] = 1;
			detection_cnt++;
			last_event_timestamp = mic1_timestamp;
		}

		if ((abs(mic2_filtered[mic2_max]) > THRESH_EVENT) && (sample_based_time - mic2_timestamp > EVENT_DB_TIME))
		{
			// account for sample delay later
			mic2_timestamp = prev_cnt * TIME_PERIOD + mic2_max * SAMPLE_PERIOD * DECIMATION_M;
			mic_detected_event[2] = 1;
			detection_cnt++;
			last_event_timestamp = mic2_timestamp;
		}

		/* Testing
		detection_cnt = 3;
		mic0_timestamp = sqrt(2) / SPEED_OF_SOUND;
		mic1_timestamp = 1.0 / SPEED_OF_SOUND;
		mic2_timestamp = 0;
		mic_detected_event[0] = 1;
		mic_detected_event[1] = 1;
		mic_detected_event[2] = 1;
		*/

		if (detection_cnt >= 3)
		{
			if (detection_cnt == 3 && mic_detected_event[0] && mic_detected_event[1] && mic_detected_event[2])
			{
				// GOOD state
				// mic0 is the reference
				float mic1_delay = mic1_timestamp - mic0_timestamp;
				float mic2_delay = mic2_timestamp - mic0_timestamp;

				// x, y coordinates of event
				union {
					float coords_f[2];			// (x, y)
					uint8_t serial[8];
				} coords;

				// initial guess
				coords.coords_f[0] = 0.23;
				coords.coords_f[1] = 0.38;

				compute_event_pos(&coords.coords_f[0], &coords.coords_f[1], MIC0_XPOS, MIC0_YPOS, MIC1_XPOS, MIC1_YPOS, MIC2_XPOS, MIC2_YPOS, mic1_delay, mic2_delay);
				uart2_dma1_write(8, coords.serial);
			}
			else
			{
				// ERROR state
			}

			// reset event detection
			detection_cnt = 0;
			mic_detected_event[0] = 0;
			mic_detected_event[1] = 0;
			mic_detected_event[2] = 0;
		}
		else if (detection_cnt != 0 && sample_based_time - last_event_timestamp > TIMEOUT_S)
		{
			detection_cnt = 0;
			mic_detected_event[0] = 0;
			mic_detected_event[1] = 0;
			mic_detected_event[2] = 0;
		}


		sample_cnt += SAMPLE_SIZE;

//		GPIOA->ODR &= ~GPIO_ODR_OD8;
	}
}

void sysclock_config(void)
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

void adc1_dma_config(void)
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

	// channel 0 (PA0 --> ADC1_IN0) is first conversion
	ADC1->SQR3 &= ~ADC_SQR3_SQ1;

	// channel 1 (PA1 --> ADC1_IN1) is second conversion
	ADC1->SQR3 &= ~ADC_SQR3_SQ2;
	ADC1->SQR3 |= ADC_SQR3_SQ2_0;

	// channel 4 (PA4 --> ADC1_IN4) is third conversion
	ADC1->SQR3 &= ~ADC_SQR3_SQ3;
	ADC1->SQR3 |= ADC_SQR3_SQ3_2;


	/* DMA2 Channel 0, Stream 0 --> ADC1 */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;		// enable DMA2 clock

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;		// disable stream
	while (DMA2_Stream0->CR & DMA_SxCR_EN);	// wait for stream to disable

	DMA2_Stream0->PAR = (uint32_t)&(ADC1->DR);// peripheral address
	DMA2_Stream0->M0AR = (uint32_t)stream0;	// destination memory address (CT = 0)
	DMA2_Stream0->M1AR = (uint32_t)stream1;	// destination memory address (CT = 1)
	DMA2_Stream0->NDTR = BLOCK_SIZE;		// number of units to be transmitted

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

void tim2_trig_config(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;				// enable TIM2 clock

	TIM2->CR1 &= ~TIM_CR1_DIR;			// upcounting
	TIM2->PSC = 0;						// /1 prescaler
	TIM2->ARR = 2099;					// 40kHz frequency

	TIM2->CR2 |= TIM_CR2_MMS_1;
	TIM2->CR2 &= ~(TIM_CR2_MMS_0 | TIM_CR2_MMS_2);		// update event as TRGO

	TIM2->EGR |= TIM_EGR_UG;			// generate update event

	TIM2->CR1 |= TIM_CR1_CEN;			// enable counter
}

void tim5_time_config(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;				// enable TIM2 clock

	TIM5->CR1 &= ~TIM_CR1_DIR;			// upcounting
	TIM5->PSC = 839;					// 100kHz frequency
	TIM5->ARR = 0xffffffff;

	TIM5->EGR |= TIM_EGR_UG;			// generate update event

	TIM5->CR1 |= TIM_CR1_CEN;			// enable counter
}

void stream_splice(void)
{
	for (uint32_t i = 0; i < BLOCK_SIZE; i += 3)
	{
		uint32_t ind = i / 3;

		// DMA is targeting M1AR ---> read from M0AR memory
		// TO DO: check ranges of mic0
		if (dma_tgt)
		{
			mic0_samp[ind] = ((int16_t)stream0[i] - 2048) << 4;
			mic1_samp[ind] = ((int16_t)stream0[i+1] - 2048) << 4;
			mic2_samp[ind] = ((int16_t)stream0[i+2] - 2048) << 4;
		}
		else
		{
			mic0_samp[ind] = ((int16_t)stream1[i] - 2048) << 4;
			mic1_samp[ind] = ((int16_t)stream1[i+1] - 2048) << 4;
			mic2_samp[ind] = ((int16_t)stream1[i+2] - 2048) << 4;
		}
	}
}

void find_filter_peaks(q15_t * mic0_filtered, q15_t * mic1_filtered, q15_t * mic2_filtered,
						uint16_t * mic0_ind, uint16_t * mic1_ind, uint16_t * mic2_ind)
{
	*mic0_ind = 0;
	*mic1_ind = 0;
	*mic2_ind = 0;
	for (uint32_t i = 1; i < DECIMATION_SIZE; i++)
	{
		if (abs(mic0_filtered[i]) > abs(mic0_filtered[*mic0_ind]))
		{
			*mic0_ind = i;
		}
		if (abs(mic1_filtered[i]) > abs(mic1_filtered[*mic1_ind]))
		{
			*mic1_ind = i;
		}
		if (abs(mic2_filtered[i]) > abs(mic2_filtered[*mic2_ind]))
		{
			*mic2_ind = i;
		}
	}
}

void compute_event_pos(float * x, float * y, float mic0_x, float mic0_y,
					   float mic1_x, float mic1_y, float mic2_x, float mic2_y,
					   float mic1_delay, float mic2_delay)
{

	for (uint8_t i = 0; i < 10; i++)
	{
		// distances from mic1 and mic2 to mic0
		float d10 = SPEED_OF_SOUND * mic1_delay;
		float d20 = SPEED_OF_SOUND * mic2_delay;

		// compute radii of guesses
		float r0 = sqrtf(powf(*x - mic0_x, 2) + powf(*y - mic0_y, 2));
		float r1 = sqrtf(powf(*x - mic1_x, 2) + powf(*y - mic1_y, 2));
		float r2 = sqrtf(powf(*x - mic2_x, 2) + powf(*y - mic2_y, 2));

		if (r0 == 0 || r1 == 0 || r2 == 0) break;

		// compute residuals (error)
		// [f]
		float res1 = r1 - r0 - d10;
		float res2 = r2 - r0 - d20;

		// create Jacobian
		float j11 = (*x - mic1_x) / r1 - (*x - mic0_x) / r0;
		float j12 = (*y - mic1_y) / r1 - (*y - mic0_y) / r0;
		float j21 = (*x - mic2_x) / r2 - (*x - mic0_x) / r0;
		float j22 = (*y - mic2_y) / r2 - (*y - mic0_y) / r0;

		// ([J]^T)[J]
		float prod11 = j11 * j11 + j21 * j21;
		float prod12 = j11 * j12 + j21 * j22;
		float prod21 = j12 * j11 + j22 * j21;
		float prod22 = j12 * j12 + j22 * j22;

		// ([J]^T)[f]
		float g1 = j11 * res1 + j21 * res2;
		float g2 = j12 * res1 + j22 * res2;

		// solve system
		// ([J]^T)[J]delta = -([J]^T)[f]
		// delta = inv(([J]^T)[J]) * (-([J]^T)[f])
		float det = prod11 * prod22 - prod12 * prod21;
		if (fabsf(det) < 1e-6) break;

		float dx = (-prod22 * g1 + prod12 * g2) / det;
		float dy = (prod21 * g1 - prod11 * g2) / det;

		*x += dx;
		*y += dy;

		if (sqrtf(dx * dx + dy * dy) < 1e-5) break;
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
