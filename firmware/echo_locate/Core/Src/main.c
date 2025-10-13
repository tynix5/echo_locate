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

#define NUM_FILTER_TAPS		48
#define BLOCK_SIZE			144								// Total samples from ADC before switching DMA targets
#define SAMPLE_SIZE			BLOCK_SIZE / 3					// Samples per microphone before switching DMA targets
#define DECIMATION_M		4								// Take every 4th sample
#define DECIMATION_SIZE		SAMPLE_SIZE / DECIMATION_M

#define WINDOW_SIZE			7
#define BUFFER_SIZE			SAMPLE_SIZE * WINDOW_SIZE

#define CORRELATION_SIZE	SAMPLE_SIZE * 4 - 1				// length of correlation sequence

#define ENERGY_THRESH		2000							// Detected event energy threshold
#define TIMEOUT_S			0.05							// Timeout before resetting microphone detections
#define EVENT_DB_TIME		0.05							// wait 50ms between position updates

#define SAMPLE_FREQ			40000							// ADC group sample rate
#define SAMPLE_PERIOD		1.0 / SAMPLE_FREQ
#define TIME_FREQ			100000							// Global timer frequency
#define TIME_PERIOD			1.0 / TIME_FREQ

#define SPEED_OF_SOUND		343								// speed is in meters/second

// distances in meters
#define MIC0_XPOS			0
#define MIC0_YPOS			0
#define MIC1_XPOS			0
#define MIC1_YPOS			1
#define MIC2_XPOS			1
#define MIC2_YPOS			1

#define MAX_TDOA			0.005							// maximum time difference (5 ms)
															// 0.005s * 343 m/s > sqrt(2) meters


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

*/



q15_t const ftaps_q15[NUM_FILTER_TAPS] = {
		427, 682, 874, 755, 246, -527,
		-1261, -1613, -1405, -775, -149, -25,
		-669, -1885, -3025, -3280, -2107, 416,
		3530, 6090, 7078, 6090, 3530, 416,
		-2107 -3280, -3025, -1885, -669, -25,
		-149, -775, -1405, -1613, -1261, -527,
		246, 755, 874, 682, 427, 0
};

//arm_fir_decimate_instance_q15 hfir0, hfir1, hfir2;
arm_fir_instance_q15 hfir0, hfir1, hfir2;

uint16_t stream0[BLOCK_SIZE], stream1[BLOCK_SIZE];									// raw data streams from DMA buffers
q15_t mic0_samp[SAMPLE_SIZE], mic1_samp[SAMPLE_SIZE], mic2_samp[SAMPLE_SIZE];		// spliced microphone sample streams

uint8_t dma_tgt = 0;				// M0AR written to first

uint8_t mic_detected_event[3] = {0};

/* Configure system clock for 84 MHz */
void sysclock_config(void);
/* Configure ADC1 to trigger via TIM2, and sample normal group ADC0, ADC1, and ADC2, streaming to DMA2 in double-buffer mode */
void adc1_dma_config(void);
/* 40 kHz trigger timer for ADC1 */
void tim2_trig_config(void);
/* 100 kHz global timer */
void tim5_time_config(void);
/* Split DMA stream into separate microphone streams, converting from biased uint16_t to q15_t */
void stream_splice(void);
/* Compute x and y coordinates of event using trilateration */
void compute_event_pos(float * x, float * y, float mic0_x, float mic0_y,
					   float mic1_x, float mic1_y, float mic2_x, float mic2_y,
					   float mic1_delay, float mic2_delay);

int main(void)
{

	sysclock_config();

	q15_t mic0_state[NUM_FILTER_TAPS + SAMPLE_SIZE - 1];
	q15_t mic1_state[NUM_FILTER_TAPS + SAMPLE_SIZE - 1];
	q15_t mic2_state[NUM_FILTER_TAPS + SAMPLE_SIZE - 1];

	/*
	arm_fir_decimate_init_q15(&hfir0, NUM_FILTER_TAPS, DECIMATION_M, ftaps_q15, mic0_state, SAMPLE_SIZE);
	arm_fir_decimate_init_q15(&hfir1, NUM_FILTER_TAPS, DECIMATION_M, ftaps_q15, mic1_state, SAMPLE_SIZE);
	arm_fir_decimate_init_q15(&hfir2, NUM_FILTER_TAPS, DECIMATION_M, ftaps_q15, mic2_state, SAMPLE_SIZE);
	*/
	arm_fir_init_q15(&hfir0, NUM_FILTER_TAPS, ftaps_q15, mic0_state, SAMPLE_SIZE);
	arm_fir_init_q15(&hfir1, NUM_FILTER_TAPS, ftaps_q15, mic1_state, SAMPLE_SIZE);
	arm_fir_init_q15(&hfir2, NUM_FILTER_TAPS, ftaps_q15, mic2_state, SAMPLE_SIZE);


	// sampled, decimated, and filtered microphone streams
//	q15_t mic0_filt[DECIMATION_SIZE];
//	q15_t mic1_filt[DECIMATION_SIZE];
//	q15_t mic2_filt[DECIMATION_SIZE];
	q15_t mic0_filt[SAMPLE_SIZE];
	q15_t mic1_filt[SAMPLE_SIZE];
	q15_t mic2_filt[SAMPLE_SIZE];

	uart2_set_fcpu(84000000);
	uart2_dma1_config(115200, USART_DATA_8, USART_STOP_1);

	tim5_time_config();

	tim2_trig_config();
	adc1_dma_config();

	GPIOA->MODER &= ~GPIO_MODER_MODER10;
	GPIOA->MODER |= GPIO_MODER_MODER10_0;

	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;

	float mic0_timestamp = 0, mic1_timestamp = 0, mic2_timestamp = 0;
	float prev_timestamp = 0;

	uint8_t detection_cnt = 0;						// # of microphones that have detected an event

	uint32_t prev_ticks = 0, ticks = 0;

	q15_t mic0_filt_max, mic1_filt_max, mic2_filt_max;		// holds max value in filtered samples
	uint32_t mic0_max_ind, mic1_max_ind, mic2_max_ind;		// holds indices of max values in filtered samples (0-DECIMATION_SIZE)

	q15_t mic0_buff[BUFFER_SIZE], mic1_buff[BUFFER_SIZE], mic2_buff[BUFFER_SIZE];

	uint8_t ind = 0;
	uint32_t mic0_event_ind, mic1_event_ind, mic2_event_ind;

	while (1)
	{

		GPIOA->ODR |= GPIO_ODR_OD10;
		while (!!(DMA2_Stream0->CR & DMA_SxCR_CT) == dma_tgt);		// wait for stream to complete
		prev_ticks = ticks;
		ticks = TIM5->CNT;
		GPIOA->ODR &= ~GPIO_ODR_OD10;
		dma_tgt = !dma_tgt;											// switch DMA targets
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;			// clear transfer complete and half complete flag

		stream_splice();

		GPIOA->ODR |= GPIO_ODR_OD8;

		// Cross correlation instead of filtering?
		/*
		arm_fir_decimate_fast_q15(&hfir0, mic0_samp, mic0_filt, SAMPLE_SIZE);
		arm_fir_decimate_fast_q15(&hfir1, mic1_samp, mic1_filt, SAMPLE_SIZE);
		arm_fir_decimate_fast_q15(&hfir2, mic2_samp, mic2_filt, SAMPLE_SIZE);
		*/

		arm_fir_fast_q15(&hfir0, mic0_samp, mic0_filt, SAMPLE_SIZE);
		arm_fir_fast_q15(&hfir1, mic1_samp, mic1_filt, SAMPLE_SIZE);
		arm_fir_fast_q15(&hfir2, mic2_samp, mic2_filt, SAMPLE_SIZE);

		arm_copy_q15(mic0_samp, mic0_filt + ind * SAMPLE_SIZE, SAMPLE_SIZE);
		arm_copy_q15(mic1_samp, mic1_filt + ind * SAMPLE_SIZE, SAMPLE_SIZE);
		arm_copy_q15(mic2_samp, mic2_filt + ind * SAMPLE_SIZE, SAMPLE_SIZE);



		// find filter peaks and their indices
		/*
		arm_absmax_q15(mic0_filt, DECIMATION_SIZE, &mic0_filt_max, &mic0_max_ind);
		arm_absmax_q15(mic1_filt, DECIMATION_SIZE, &mic1_filt_max, &mic1_max_ind);
		arm_absmax_q15(mic2_filt, DECIMATION_SIZE, &mic2_filt_max, &mic2_max_ind);
		*/
		arm_absmax_q15(mic0_filt, SAMPLE_SIZE, &mic0_filt_max, &mic0_max_ind);
		arm_absmax_q15(mic1_filt, SAMPLE_SIZE, &mic1_filt_max, &mic1_max_ind);
		arm_absmax_q15(mic2_filt, SAMPLE_SIZE, &mic2_filt_max, &mic2_max_ind);


		// reference time is based on # of samples
		float ref_time = prev_ticks * TIME_PERIOD;

		// if a new event has been detected, update time stamps and event count
		if ((abs(mic0_filt_max) > ENERGY_THRESH) && (ref_time - mic0_timestamp > EVENT_DB_TIME))
		{
//			mic0_timestamp = ref_time + mic0_max_ind * SAMPLE_PERIOD * DECIMATION_M;
			mic0_timestamp = ref_time + mic0_max_ind * SAMPLE_PERIOD;
			mic0_event_ind = mic0_max_ind + ind * SAMPLE_SIZE;
			mic_detected_event[0] = 1;
			detection_cnt++;
			prev_timestamp = mic0_timestamp;
		}

		if ((abs(mic1_filt_max) > ENERGY_THRESH) && (ref_time - mic1_timestamp > EVENT_DB_TIME))
		{
			// account for sample delay later
//			mic1_timestamp = ref_time + mic1_max_ind * SAMPLE_PERIOD * DECIMATION_M;
			mic1_timestamp = ref_time + mic1_max_ind * SAMPLE_PERIOD;
			mic1_event_ind = mic1_max_ind + ind * SAMPLE_SIZE;
			mic_detected_event[1] = 1;
			detection_cnt++;
			prev_timestamp = mic1_timestamp;
		}

		if ((abs(mic2_filt_max) > ENERGY_THRESH) && (ref_time - mic2_timestamp > EVENT_DB_TIME))
		{
			// account for sample delay later
//			mic2_timestamp = ref_time + mic2_max_ind * SAMPLE_PERIOD * DECIMATION_M;
			mic2_timestamp = ref_time + mic2_max_ind * SAMPLE_PERIOD;
			mic2_event_ind = mic2_max_ind + ind * SAMPLE_SIZE;
			mic_detected_event[2] = 1;
			detection_cnt++;
			prev_timestamp = mic2_timestamp;
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

				// take 48 samples centered around each event index
				q15_t mic0_corr_buff[SAMPLE_SIZE], mic1_corr_buff[SAMPLE_SIZE], mic2_corr_buff[SAMPLE_SIZE];
				// output cross correlation sequences
				q15_t corr_mic01[CORRELATION_SIZE], corr_mic02[CORRELATION_SIZE];

				uint32_t corr_peak_ind_01, corr_peak_ind_02;
				q15_t corr_peak_01, corr_peak_02;
				arm_correlate_fast_q15(mic0_corr_buff, SAMPLE_SIZE, mic1_corr_buff, SAMPLE_SIZE, corr_mic01);
				arm_correlate_fast_q15(mic0_corr_buff, SAMPLE_SIZE, mic2_corr_buff, SAMPLE_SIZE, corr_mic02);
				arm_absmax_q15(corr_mic01, CORRELATION_SIZE, &corr_peak_01, &corr_peak_ind_01);
				arm_absmax_q15(corr_mic02, CORRELATION_SIZE, &corr_peak_02, &corr_peak_ind_02);

				float mic1_delay = (corr_peak_ind_01 - (SAMPLE_SIZE - 1)) * SAMPLE_PERIOD;
				float mic2_delay = (corr_peak_ind_02 - (SAMPLE_SIZE - 1)) * SAMPLE_PERIOD;

				// mic0 is the reference
//				float mic1_delay = mic1_timestamp - mic0_timestamp;
//				float mic2_delay = mic2_timestamp - mic0_timestamp;

				if (abs(mic1_delay) < MAX_TDOA && abs(mic2_delay) < MAX_TDOA)
				{
					// x, y coordinates of event
					union {
						float coords_f[2];			// (x, y)
						uint8_t serial[8];
					} coords;

					// initial guess
					if (mic1_delay > 0 && mic2_delay > 0)
					{
						coords.coords_f[0] = 0.1;
						coords.coords_f[1] = 0.1;
					}
					else if (mic1_delay < 0 && mic2_delay > 0)
					{
						coords.coords_f[0] = 0.9;
						coords.coords_f[1] = 0.1;
					}
					else if (mic1_delay > 0 && mic2_delay < 0)
					{
						coords.coords_f[0] = 0.7;
						coords.coords_f[1] = 0.7;
					}
					else
					{
						coords.coords_f[0] = 0.8;
						coords.coords_f[1] = 0.8;
					}

					compute_event_pos(&coords.coords_f[0], &coords.coords_f[1], MIC0_XPOS,
							MIC0_YPOS, MIC1_XPOS, MIC1_YPOS, MIC2_XPOS, MIC2_YPOS, mic1_delay, mic2_delay);
					uart2_dma1_write(8, coords.serial);
				}
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
		else if (detection_cnt != 0 && ref_time - prev_timestamp > TIMEOUT_S)
		{
			detection_cnt = 0;
			mic_detected_event[0] = 0;
			mic_detected_event[1] = 0;
			mic_detected_event[2] = 0;
		}
//		GPIOA->ODR &= ~GPIO_ODR_OD8;
		if (++ind == WINDOW_SIZE)
			ind = 0;
		GPIOA->ODR &= ~GPIO_ODR_OD8;

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
			// convert from uint16_t in range [0, 4095] from ADC to q15_t range [-1, 1]
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

void compute_event_pos(float * x, float * y, float mic0_x, float mic0_y,
					   float mic1_x, float mic1_y, float mic2_x, float mic2_y,
					   float mic1_delay, float mic2_delay)
{

	const float max_step = 0.2;			// maximum dx/dy change per iteration in meters
	const float eps = 1e-6;

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
		float j11 = (*x - mic1_x) / (r1 + eps) - (*x - mic0_x) / (r0 + eps);
		float j12 = (*y - mic1_y) / (r1 + eps) - (*y - mic0_y) / (r0 + eps);
		float j21 = (*x - mic2_x) / (r2 + eps) - (*x - mic0_x) / (r0 + eps);
		float j22 = (*y - mic2_y) / (r2 + eps) - (*y - mic0_y) / (r0 + eps);

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

		if (dx > max_step)			dx = max_step;
		else if (dx < -max_step)	dx = -max_step;
		if (dy > max_step)			dy = max_step;
		else if (dy < -max_step)	dy = -max_step;

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
