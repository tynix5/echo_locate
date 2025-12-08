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

#define NUM_FILTER_TAPS		48
#define BLOCK_SIZE			1440							// total samples from ADC before switching DMA targets
#define SAMPLE_SIZE			(BLOCK_SIZE / 3)				// samples per microphone before switching DMA targets

#define SAMPLES_BEFORE_DET	10								// # of samples used before peak detection in cross correlation
#define SAMPLES_AFTER_DET	189								// # of samples needed after first peak detection to compute cross correlation

// CORR_IN_SIZE * SAMPLE_PERIOD * SPEED_OF_SOUND is maximum distance that can be calculated
#define CORR_IN_SIZE		(SAMPLES_BEFORE_DET + SAMPLES_AFTER_DET + 1)		// size of input window for correlation
#define WINDOW_SIZE			2
#define BUFFER_SIZE			WINDOW_SIZE * SAMPLE_SIZE		// buffer for previous microphone samples
#define CORR_OUT_SIZE		2 * CORR_IN_SIZE - 1			// correlated sequence length (convolution)

#define ENERGY_THRESH		8000							// detected event energy threshold
#define UPDATE_PERIOD		0.025							// delay between position updates
#define DETECTION_TIMEOUT	CORR_IN_SIZE * SAMPLE_PERIOD	// maximum allowable delay between consecutive peak detections before timeout
#define MAX_TDOA			DETECTION_TIMEOUT

#define SAMPLE_FREQ			40000.0f						// ADC group sample rate
#define SAMPLE_PERIOD		(1.0f / SAMPLE_FREQ)
#define TIME_FREQ			100000.0f						// global timer frequency
#define TIME_PERIOD			(1.0f / TIME_FREQ)

#define SPEED_OF_SOUND		343.0f							// m/s

#define NUM_MICS			3

// distances in meters
#define MIC0_XPOS			0
#define MIC0_YPOS			0
#define MIC1_XPOS			0
#define MIC1_YPOS			1
#define MIC2_XPOS			1
#define MIC2_YPOS			1

#if CORR_IN_SIZE > BUFFER_SIZE
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
uint8_t compute_event_pos(float * x, float * y, float mic0_x, float mic0_y,
					   float mic1_x, float mic1_y, float mic2_x, float mic2_y,
					   float mic1_delay, float mic2_delay);
/* compute_event_pos helper function */
float clamp(float in, float abs_max);

int main(void)
{

	sysclock_config();

	static q15_t mic0_state[NUM_FILTER_TAPS + SAMPLE_SIZE - 1];
	static q15_t mic1_state[NUM_FILTER_TAPS + SAMPLE_SIZE - 1];
	static q15_t mic2_state[NUM_FILTER_TAPS + SAMPLE_SIZE - 1];

	arm_fir_init_q15(&hfir0, NUM_FILTER_TAPS, ftaps_q15, mic0_state, SAMPLE_SIZE);
	arm_fir_init_q15(&hfir1, NUM_FILTER_TAPS, ftaps_q15, mic1_state, SAMPLE_SIZE);
	arm_fir_init_q15(&hfir2, NUM_FILTER_TAPS, ftaps_q15, mic2_state, SAMPLE_SIZE);

	static q15_t mic0_buff[BUFFER_SIZE], mic1_buff[BUFFER_SIZE], mic2_buff[BUFFER_SIZE];

	// sampled and filtered microphone streams
	static q15_t mic0_filt[SAMPLE_SIZE], mic1_filt[SAMPLE_SIZE], mic2_filt[SAMPLE_SIZE];

	uart2_set_fcpu(84000000);
	uart2_dma1_config(115200, USART_DATA_8, USART_STOP_1);

	tim5_time_config();

	tim2_trig_config();
	adc1_dma_config();

	/*
	GPIOA->MODER &= ~GPIO_MODER_MODER10;
	GPIOA->MODER |= GPIO_MODER_MODER10_0;

	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	*/

	uint32_t prev_ticks = 0, ticks = 0;

	uint8_t window_ind = 0;

	uint8_t mic_detected_event[NUM_MICS] = {0};				// has microphone detected an event?
	int32_t mic_first_peak[NUM_MICS] = {0};					// sample index of first peak in this sound event
	uint32_t mic_last_det[NUM_MICS] = {0};					// last time microphone peak was detected

	float last_det_update = 0;								// last time a microphone detected a peak above energy threshold
	float last_pos_update = 0;								// last time position was updated
	uint8_t detected_event = 0;								// have all microphones detected an event?
	uint32_t ref_sample = 0;
	uint32_t samples = 0;

	while (1)
	{
		// echo_locate
		// 1. Wait for DMA interrupt indicating stream complete
		// 2. Splice DMA stream into three separate microphone streams
		// 3. Filter raw microphone streams
		// 4. Copy filtered streams to buffers
		// 5. Find peaks in microphone streams
		// 6. If a stream has a peak that exceeds the threshold, indicate detection
		// 7. After all three microphones detected a peak above threshold, ensure they fall in a reasonable window
		// 8. Take the earliest peak as a reference sample
		// 9. Copy samples from right before reference sample to the farthest sample peak possible in a correlation buffer
		// 10. Cross correlate mic1 and mic2 correlation buffers with mic0 to get delays
		// 11. Use nonlinear least squares to estimate sound origin

//		GPIOA->ODR |= GPIO_ODR_OD10;
		while (!!(DMA2_Stream0->CR & DMA_SxCR_CT) == dma_tgt);		// wait for stream to complete
		prev_ticks = ticks;
		ticks = TIM5->CNT;
		samples += SAMPLE_SIZE;
//		GPIOA->ODR &= ~GPIO_ODR_OD10;
		dma_tgt = !dma_tgt;											// switch DMA targets
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;			// clear transfer complete and half complete flag

//		GPIOA->ODR |= GPIO_ODR_OD8;
		stream_splice();

		// sanitize microphone streams with bandpass filter
		arm_fir_fast_q15(&hfir0, mic0_samp, mic0_filt, SAMPLE_SIZE);
		arm_fir_fast_q15(&hfir1, mic1_samp, mic1_filt, SAMPLE_SIZE);
		arm_fir_fast_q15(&hfir2, mic2_samp, mic2_filt, SAMPLE_SIZE);

		// copy data to window buffers for cross correlation
		arm_copy_q15(mic0_filt, mic0_buff + window_ind * SAMPLE_SIZE, SAMPLE_SIZE);
		arm_copy_q15(mic1_filt, mic1_buff + window_ind * SAMPLE_SIZE, SAMPLE_SIZE);
		arm_copy_q15(mic2_filt, mic2_buff + window_ind * SAMPLE_SIZE, SAMPLE_SIZE);

		// reference time is based on # ticks since last DMA buffer started
		float ref_time = prev_ticks * TIME_PERIOD;

		// wait for a while to update sound position (for constant noise)
		if (!detected_event && ref_time - last_pos_update > UPDATE_PERIOD)
		{
			q15_t mic_filt_peak[NUM_MICS];
			uint32_t mic_filt_peak_ind[NUM_MICS];

			// find peaks in filtered streams
			arm_absmax_q15(mic0_filt, SAMPLE_SIZE, &mic_filt_peak[0], &mic_filt_peak_ind[0]);
			arm_absmax_q15(mic1_filt, SAMPLE_SIZE, &mic_filt_peak[1], &mic_filt_peak_ind[1]);
			arm_absmax_q15(mic2_filt, SAMPLE_SIZE, &mic_filt_peak[2], &mic_filt_peak_ind[2]);

			for (uint8_t i = 0; i < NUM_MICS; i++)
			{
				// if this microphone has not detected a peak yet, but a new sample meets the energy threshold...
				if (!mic_detected_event[i] && mic_filt_peak[i] > ENERGY_THRESH)
				{
					mic_first_peak[i] = mic_filt_peak_ind[i] + window_ind * SAMPLE_SIZE;			// save index of first peak for this sound event
					mic_last_det[i] = ref_time + mic_filt_peak_ind[i] * SAMPLE_PERIOD;				// save time of event detection
					last_det_update = mic_last_det[i];												// update most recent detection time
					mic_detected_event[i] = 1;

					// if all microphones have detected valid events, compute sound origin
					if (mic_detected_event[0] && mic_detected_event[1] && mic_detected_event[2])
					{
						// reference sample is the first peak (minimum index)
						if (mic_first_peak[0] <= mic_first_peak[1] && mic_first_peak[0] <= mic_first_peak[2])
							ref_sample = mic_first_peak[0];
						else if (mic_first_peak[1] <= mic_first_peak[0] && mic_first_peak[1] <= mic_first_peak[2])
							ref_sample = mic_first_peak[1];
						else
							ref_sample = mic_first_peak[2];

						detected_event = 1;
					}
				}
				// if this microphone has already detected a peak, and last time microphone peak was detected is not possible (past max distance)...
				// (this will happen if one microphone detects a faint sound, but others do not)
				else if (mic_detected_event[i] && ref_time - last_det_update > DETECTION_TIMEOUT)
				{
					// clear all detections and restart
					detected_event = 0;
					mic_detected_event[0] = 0;
					mic_detected_event[1] = 0;
					mic_detected_event[2] = 0;
				}
			}
		}
		else if (detected_event && ref_sample + SAMPLES_AFTER_DET < samples) 				// make sure enough samples have been taken after peak detected
		{
			// take 10 samples starting from before first peak, then 189 after first peak
			static q15_t mic0_corr_buff[CORR_IN_SIZE], mic1_corr_buff[CORR_IN_SIZE], mic2_corr_buff[CORR_IN_SIZE];
			// output cross correlation sequences
			static q15_t corr_mic01[CORR_OUT_SIZE], corr_mic02[CORR_OUT_SIZE];

			int32_t start_sample = ref_sample - SAMPLES_BEFORE_DET;
			int32_t stop_sample = ref_sample + SAMPLES_AFTER_DET;

			if (start_sample > -1 && stop_sample < BUFFER_SIZE)			// full length of correlation input is contiguous
			{
				arm_copy_q15(mic0_buff + start_sample, mic0_corr_buff, CORR_IN_SIZE);
				arm_copy_q15(mic1_buff + start_sample, mic1_corr_buff, CORR_IN_SIZE);
				arm_copy_q15(mic2_buff + start_sample, mic2_corr_buff, CORR_IN_SIZE);
			}
			else 														// if correlation length wraps, copy separately
			{
				int32_t curr_sample;									// should always be +

				if (start_sample < 0)
					curr_sample = (int32_t)BUFFER_SIZE + start_sample;			// if start is on other end of buffer, move pointer back
				else
					curr_sample = start_sample;

				for (uint32_t i = 0; i < CORR_IN_SIZE; i++)
				{
					mic0_corr_buff[i] = mic0_buff[curr_sample];
					mic1_corr_buff[i] = mic1_buff[curr_sample];
					mic2_corr_buff[i] = mic2_buff[curr_sample];

					if (++curr_sample == BUFFER_SIZE)					// wrap pointer back to front on overflow
						curr_sample = 0;
				}
			}

			// Run cross correlation between mic 0 and mic 1 then mic 0 and mic 2
			uint32_t corr_peak_01_ind, corr_peak_02_ind;
			q15_t corr_peak_01, corr_peak_02;

			arm_fill_q15(0, corr_mic01, CORR_OUT_SIZE);
			arm_fill_q15(0, corr_mic02, CORR_OUT_SIZE);
			arm_correlate_q15(mic0_corr_buff, CORR_IN_SIZE, mic1_corr_buff, CORR_IN_SIZE, corr_mic01);
			arm_correlate_q15(mic0_corr_buff, CORR_IN_SIZE, mic2_corr_buff, CORR_IN_SIZE, corr_mic02);

			// find peaks in cross correlation output to determine sample delay
			arm_absmax_q15(corr_mic01, CORR_OUT_SIZE, &corr_peak_01, &corr_peak_01_ind);
			arm_absmax_q15(corr_mic02, CORR_OUT_SIZE, &corr_peak_02, &corr_peak_02_ind);

			// calculate time delay based on sample difference --> 0 sample delay rests at middle of cross correlation sequence SAMPLE_SIZE - 1
			float mic1_delay = -((int32_t) corr_peak_01_ind - (CORR_IN_SIZE - 1)) * SAMPLE_PERIOD;
			float mic2_delay = -((int32_t) corr_peak_02_ind - (CORR_IN_SIZE - 1)) * SAMPLE_PERIOD;

			if (fabs(mic1_delay) < MAX_TDOA && fabs(mic2_delay) < MAX_TDOA)
			{
				// x, y coordinates of event
				union {
					float coords_f[2];			// (x, y)
					uint8_t serial[8];
				} coords;

				// place initial guess at centroid
				coords.coords_f[0] = (MIC0_XPOS + MIC1_XPOS + MIC2_XPOS) / 3.0f;
				coords.coords_f[1] = (MIC0_YPOS + MIC1_YPOS + MIC2_YPOS) / 3.0f;

				uint8_t valid = compute_event_pos(&coords.coords_f[0], &coords.coords_f[1], MIC0_XPOS,
						MIC0_YPOS, MIC1_XPOS, MIC1_YPOS, MIC2_XPOS, MIC2_YPOS, mic1_delay, mic2_delay);

				if (!valid || coords.coords_f[0] > 1.2f || coords.coords_f[0] < -0.2f || coords.coords_f[1] > 1.2f || coords.coords_f[1] < -0.2f)
				{
					// if NLLS doesn't converge or values are garbage, error, but indicate sound detected
					coords.coords_f[0] = -1.0f;
					coords.coords_f[1] = -1.0f;
					uart2_dma1_write(8, coords.serial);
				}
				else
				{
					uart2_dma1_write(8, coords.serial);
				}

			}

			// reset event detection
			for (uint8_t i = 0; i < NUM_MICS; i++)
			{
				mic_detected_event[i] = 0;
			}
			detected_event = 0;
			last_pos_update = TIM5->CNT * TIME_PERIOD;
		}

		if (++window_ind == WINDOW_SIZE)
			window_ind = 0;

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
