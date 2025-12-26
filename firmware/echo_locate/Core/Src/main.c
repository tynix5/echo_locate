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
#include "arm_math.h"
#include <stdlib.h>
#include <stdio.h>

#define N_BP_TAPS			42
#define N_LP_TAPS			42

#define N_BLOCK				1440							// total samples from ADC before switching DMA targets
#define N_SAMPLE			N_BLOCK / 3						// samples per microphone before switching DMA targets

#define N_WINDOW			2
#define N_BUFFER			N_WINDOW * N_SAMPLE				// buffer for previous microphone samples

#define N_FFT				256								// length of FFT for GCC-PHAT
#define N_SAMPLES_BEFORE	26								// gather samples before maximum peak to get full shape
#define N_SAMPLES_AFTER		229								// N > MAX_DIST / (SAMPLE_PERIOD * SPEED_OF_SOUND) = 165

#define ENERGY_THRESH		8000.0f							// detected event energy threshold

#define N_UPDATE_DELAY		500								// about 12.5ms between updates

#define F_SAMPLE			40000.0f						// ADC group sample rate
#define T_SAMPLE			1.0f / F_SAMPLE

#define SPEED_OF_SOUND		343.0f							// m/s

#define N_MICS				3

#define MAX_DIST			1.414							// sqrt(2) --> distance from mic0 to mic2

// distances in meters
#define MIC0_XPOS			0
#define MIC0_YPOS			0
#define MIC1_XPOS			0
#define MIC1_YPOS			1
#define MIC2_XPOS			1
#define MIC2_YPOS			1

#if N_FFT > N_BUFFER / 2
#error	"FFT input length cannot exceed half of buffer length"
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

float32_t const bandpass_taps[N_BP_TAPS] = {
		0.013024996511747034, 0.02081231010417477, 0.026685590226205227, 0.023031794500352313, 0.007520216449306763,
		-0.016090052552988288, -0.03849239728393633, -0.04921624551771023, -0.042885108349138626, -0.023664480038404784,
		-0.004541353788974114, -0.0007683969964721174, -0.020425478999019347, -0.05751122103456796, -0.09231842258036177,
		-0.10010412745245136, -0.06431405980269875, 0.012684474364821457, 0.10774048235542323, 0.18586684053806685,
		0.21601107052289345, 0.18586684053806685, 0.10774048235542323, 0.012684474364821457, -0.06431405980269875,
		-0.10010412745245136, -0.09231842258036177, -0.05751122103456796, -0.020425478999019347, -0.0007683969964721174,
		-0.004541353788974114, -0.023664480038404784, -0.042885108349138626, -0.04921624551771023, -0.03849239728393633,
		-0.016090052552988288, 0.007520216449306763, 0.023031794500352313, 0.026685590226205227, 0.02081231010417477,
		0.013024996511747034, 0
};

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 40000 Hz

* 0 Hz - 5000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 4.152902868688468 dB

* 6000 Hz - 20000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.04629176398793 dB

*/

float32_t const lowpass_taps[N_LP_TAPS] = {
		0.013140750890121828, 0.021198220810042096, 0.027604914499249664, 0.024872679012043458, 0.010750686921766485,
		-0.010955572802103634, -0.030934992390477648, -0.03878611297580051, -0.029234804504588666, -0.006549859009201667,
		0.01613627187769598, 0.02347280044823164, 0.007271895688615912, -0.026542498783174703, -0.05834333807827964,
		-0.06345738932869462, -0.025392549461722966, 0.05343715017069038, 0.1498369983207342, 0.22877080850128836,
		0.25919563096538806, 0.22877080850128836, 0.1498369983207342, 0.05343715017069038, -0.025392549461722966,
		-0.06345738932869462, -0.05834333807827964, -0.026542498783174703, 0.007271895688615912, 0.02347280044823164,
		0.01613627187769598, -0.006549859009201667, -0.029234804504588666, -0.03878611297580051, -0.030934992390477648,
		-0.010955572802103634, 0.010750686921766485, 0.024872679012043458, 0.027604914499249664, 0.021198220810042096,
		0.013140750890121828, 0
};

uint16_t stream0[N_BLOCK], stream1[N_BLOCK];										// raw data streams from DMA buffers

struct MicProc {

	arm_rfft_fast_instance_f32 hfft;					// real FFT handle
	arm_fir_instance_f32 bp_hfir;						// bandpass FIR handle
	arm_fir_instance_f32 lp_hfir;						// lowpass FIR handle
	float32_t bp_state[N_BP_TAPS + N_SAMPLE - 1];		// bandpass FIR filter state
	float32_t lp_state[N_LP_TAPS + N_SAMPLE - 1];		// lowpass FIR filter state

	float32_t raw[N_SAMPLE];							// raw samples coming from ADCs
	float32_t samples[N_SAMPLE];						// bandpassed samples received from FPGA
	float32_t envelope[N_SAMPLE];						// scaled, lowpassed, and square rooted samples
	float32_t buffer[N_BUFFER];							// running window

	float32_t fft_window[N_FFT];						// samples taken from buffer (after envelope)
	float32_t fft[N_FFT];								// FFT of fft_window (size: CMSIS-DSP reference)
	// add variables for noise mean?
};

struct MicCoord {

	float32_t x;
	float32_t y;
};


uint8_t dma_tgt = 0;				// M0AR written to first


/* Configure system clock for 84 MHz */
void sysclock_init(void);
/* Configure SPI1 to receive ADC stream from FPGA using DMA2 in double-buffer mode */
void spi1_dma2_init(void);
/* Split DMA stream into separate microphone streams, converting from biased uint16_t to q15_t */
void stream_splice(struct MicProc * mics);
/* Compute x and y coordinates of event using trilateration */
uint8_t compute_event_pos(float32_t * x, float32_t * y, struct MicCoord * mics_xy, float32_t mic1_delay, float32_t mic2_delay);
/* compute_event_pos helper function */
float32_t clamp(float32_t in, float32_t abs_max);
/* Envelope detector function */
void compute_envelope(struct MicProc * mics);
/* Threshold-search function, returns index of first value where src > thresh */
int32_t thresh_search(float32_t * src, uint32_t len, float32_t thresh);
/* simple sorting function */
void simple_sort3(int32_t * src, int32_t * dst);

int main(void)
{

	sysclock_init();

	struct MicProc mics[N_MICS];
	struct MicCoord mics_xy[N_MICS] = {{.x = MIC0_XPOS, .y = MIC0_YPOS}, {.x = MIC1_XPOS, .y = MIC1_YPOS}, {.x = MIC2_XPOS, .y = MIC2_YPOS}};

	// initiate FIRs
	for (uint32_t i = 0; i < N_MICS; i++)
	{
		arm_rfft_fast_init_256_f32(&mics[i].hfft);
		arm_fir_init_f32(&mics[i].bp_hfir, N_BP_TAPS, bandpass_taps, mics[i].bp_state, N_SAMPLE);
		arm_fir_init_f32(&mics[i].lp_hfir, N_LP_TAPS, lowpass_taps, mics[i].lp_state, N_SAMPLE);
	}

	float32_t xcorr_01_time[N_FFT], xcorr_02_time[N_FFT];
	arm_rfft_fast_instance_f32 xcorr_01_freq_hfft, xcorr_02_freq_hfft;
	arm_rfft_fast_init_256_f32(&xcorr_01_freq_hfft);
	arm_rfft_fast_init_256_f32(&xcorr_02_freq_hfft);

	uart2_set_fcpu(84000000);
	uart2_dma1_config(115200, USART_DATA_8, USART_STOP_1);

	spi1_dma2_init();

	/*
	GPIOA->MODER &= ~GPIO_MODER_MODER10;
	GPIOA->MODER |= GPIO_MODER_MODER10_0;

	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	*/


	uint8_t window_ind = 0;

	uint8_t triggered = 0;									// has a microphone detected an event?
	uint32_t ref_sample = 0;
	uint32_t samples = 0;
	uint32_t last_trigger_sample = 0;

	while (1)
	{

//		GPIOA->ODR |= GPIO_ODR_OD10;

		while (!!(DMA2_Stream0->CR & DMA_SxCR_CT) == dma_tgt);		// wait for stream to complete
		samples += N_SAMPLE;
//		GPIOA->ODR &= ~GPIO_ODR_OD10;
		dma_tgt = !dma_tgt;											// switch DMA targets
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;			// clear transfer complete and half complete flag

//		GPIOA->ODR |= GPIO_ODR_OD8;

		stream_splice(mics);

		// bandpass filter raw samples
		for (uint32_t i = 0; i < N_MICS; i++)
			arm_fir_f32(&mics[i].bp_hfir, mics[i].raw, mics[i].samples, N_SAMPLE);

		// square, lowpass, sqrt
		compute_envelope(mics);

		for (uint32_t i = 0; i < N_MICS; i++)
			arm_copy_f32(mics[i].envelope, mics[i].buffer + window_ind * N_SAMPLE, N_SAMPLE); 		// copy envelope to buffer for processing later


		if (!triggered && samples - last_trigger_sample > N_UPDATE_DELAY)			// wait for event detection after N_UPDATE_DELAY samples since last event
		{
			// select a reference sample by finding the first peak (from all mics) above energy threshold

			// find first peak above ENERGY_THRESH in envelope for each microphone
			// if no such peak exists, place -1 at that index
			int32_t thresh_ind[N_MICS], thresh_ind_sorted[N_MICS];

			for (uint32_t i = 0; i < N_MICS; i++)
				thresh_ind[i] = thresh_search(mics[i].envelope, N_SAMPLE, ENERGY_THRESH);

			simple_sort3(thresh_ind, thresh_ind_sorted);		// sort the indices of the first peaks

			// find first non -1 index, that index becomes reference sample
			for (uint32_t i = 0; i < N_MICS; i++)
			{

				if (thresh_ind_sorted[i] != -1)
				{
					ref_sample = thresh_ind_sorted[i] + window_ind * N_SAMPLE;				// calculate reference sample location in buffer
					last_trigger_sample = samples - (N_SAMPLE - thresh_ind_sorted[i]);		// compute global sample value
					triggered = 1;
					break;
				}
			}
		}

		if (triggered && samples > last_trigger_sample + N_SAMPLES_AFTER)		// wait until enough samples have been taken after peak is detected
		{
			// extract windows
			int32_t start_sample = ref_sample - N_SAMPLES_BEFORE;
			int32_t stop_sample = ref_sample + N_SAMPLES_AFTER;

			if (start_sample > 0 && stop_sample < N_BUFFER)					// if N_FFT is not wrapped in buffer...
			{
				// direct copy entire block from buffer to FFT window
				for (uint32_t i = 0; i < N_MICS; i++)
					arm_copy_f32(mics[i].buffer + start_sample, mics[i].fft_window, N_FFT);
			}
			else
			{
				int32_t curr_sample;										// should always be +

				if (start_sample < 0)
					curr_sample = (int32_t)N_BUFFER + start_sample;			// if start is on other end of buffer, move pointer back
				else
					curr_sample = start_sample;

				for (uint32_t i = 0; i < N_FFT; i++)
				{
					for (uint32_t j = 0; j < N_MICS; j++)
						mics[j].fft_window[i] = mics[j].buffer[curr_sample];	// copy buffer sample to FFT window

					if (++curr_sample == N_BUFFER)								// wrap pointer back to front on overflow
						curr_sample = 0;
				}
			}

			// compute GCC-PHAT

			// compute FFTs for each window
			for (uint32_t i = 0; i < N_MICS; i++)
				arm_rfft_fast_f32(&mics[i].hfft, mics[i].fft_window, mics[i].fft, 0);

			// FFT output in memory
			// Bin 0: Re{X[0]} (DC offset)
			// Bin 1: Re{X[128]} (Nyquist)
			// Bin 2: Re{X[1]}
			// Bin 3: Imag{X[1]
			// Bin 4: Re{X[2]}
			// Bin 5: Imag{X[2]}
			// .....
			// Bin 254: Re{X[127]}
			// Bin 255: Imag{X[127]}

			// to compute correlation in frequency domain, conjugate X[1] and X[2], then multiply with X[0]
			// Use mic 0 as reference
			// compute cross correlation between mic 0 - mic 1 and mic 0 - mic 2 (in frequency domain)
			float32_t xcorr_01_freq[N_FFT], xcorr_02_freq[N_FFT];

			// X = a + jb ---> X[0] = a, X[1] = b
			// Y = c + jd ---> Y[0] = c, Y[1] = d
			// Y* = c - jd --> Y*[0] = c, Y*[1] = -d
			// Z = X * Y* = ac - jad + jbc + bd
			// Z[0] = ac + bd
			// Z[1] = bc - ad
			for (uint32_t i = 2; i < N_FFT; i += 2)
			{
				// compute real and imaginary cross correlation values per index
				float32_t re_01 = mics[0].fft[i] * mics[1].fft[i] + mics[0].fft[i+1] * mics[1].fft[i+1];		// real
				float32_t im_01 = mics[0].fft[i+1] * mics[1].fft[i] - mics[0].fft[i] * mics[1].fft[i+1];		// imag

				float32_t re_02 = mics[0].fft[i] * mics[2].fft[i] + mics[0].fft[i+1] * mics[2].fft[i+1];		// real
				float32_t im_02 = mics[0].fft[i+1] * mics[2].fft[i] - mics[0].fft[i] * mics[2].fft[i+1];		// imag

				// apply PHAT weighting, normalizing magnitude
				float32_t mag_01 = sqrtf(re_01 * re_01 + im_01 * im_01);
				float32_t mag_02 = sqrtf(re_02 * re_02 + im_02 * im_02);

				xcorr_01_freq[i] = re_01 * (1 / mag_01);
				xcorr_01_freq[i+1] = im_01 * (1 / mag_01);

				xcorr_02_freq[i] = re_02 * (1 / mag_02);
				xcorr_02_freq[i+1] = im_02 * (1 / mag_02);
			}

			// zero-out DC and Nyquist for cleaner iFFT
			// signals have already been bandpassed and lowpassed so they should be ~= 0
			xcorr_01_freq[0] = xcorr_01_freq[1] = xcorr_02_freq[0] = xcorr_02_freq[1] = 0;

			// iFFT
			arm_rfft_fast_f32(&xcorr_01_freq_hfft, xcorr_01_freq, xcorr_01_time, 1);
			arm_rfft_fast_f32(&xcorr_02_freq_hfft, xcorr_02_freq, xcorr_02_time, 1);

			// estimate location by finding peaks in cross correlation and converting to time
			float32_t dummy;
			uint32_t max_ind_01, max_ind_02;
			arm_max_f32(xcorr_01_time, N_FFT, &dummy, &max_ind_01);
			arm_max_f32(xcorr_02_time, N_FFT, &dummy, &max_ind_02);

			float32_t mic01_delay = (float32_t) (max_ind_01 - (N_FFT / 2)) * T_SAMPLE;
			float32_t mic02_delay = (float32_t) (max_ind_02 - (N_FFT / 2)) * T_SAMPLE;

			// x, y coordinates of event
			union {
				float32_t xy[2];
				uint8_t ser[8];
			} coords;

			// place initial guess at centroid
			coords.xy[0] = (MIC0_XPOS + MIC1_XPOS + MIC2_XPOS) / 3.0f;
			coords.xy[1] = (MIC0_YPOS + MIC1_YPOS + MIC2_YPOS) / 3.0f;

			uint8_t valid = compute_event_pos(&coords.xy[0], &coords.xy[1], mics_xy, mic01_delay, mic02_delay);

			if (!valid || coords.xy[0] > 1.2f || coords.xy[0] < -0.2f || coords.xy[1] > 1.2f || coords.xy[1] < -0.2f)
			{
				// if NLLS doesn't converge or values are garbage, error, but indicate sound detected
				coords.xy[0] = -1.0f;
				coords.xy[1] = -1.0f;
				uart2_dma1_write(8, coords.ser);
			}
			else
			{
				uart2_dma1_write(8, coords.ser);
			}

			// reset event detection
			triggered = 0;
		}

		if (++window_ind == N_WINDOW)
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

void spi1_dma2_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;				// enable SPI1 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			// enable GPIOA clock

	// PA4 is SPI1_NSS (select alternate function)
	GPIOA->MODER |= GPIO_MODER_MODER4_1;
	GPIOA->MODER &= ~GPIO_MODER_MODER4_0;

	// PA5 is SPI1_SCK (AF)
	GPIOA->MODER |= GPIO_MODER_MODER5_1;
	GPIOA->MODER &= ~GPIO_MODER_MODER5_0;

	// PA6 is SPI1_MISO (AF)
//	GPIOA->MODER |= GPIO_MODER_MODER6_1;
//	GPIOA->MODER &= ~GPIO_MODER_MODER6_0;

	// PA7 is SPI1_MOSI (AF)
	GPIOA->MODER |= GPIO_MODER_MODER7_1;
	GPIOA->MODER &= ~GPIO_MODER_MODER7_0;

	// Select AF mode 05 for all SPI1 pins
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL4_0 | GPIO_AFRL_AFRL4_2;
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL4_1 | GPIO_AFRL_AFRL4_3);

	GPIOA->AFR[0] |= GPIO_AFRL_AFRL5_0 | GPIO_AFRL_AFRL5_2;
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5_1 | GPIO_AFRL_AFRL5_3);

//	GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_0 | GPIO_AFRL_AFRL6_2;
//	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL6_1 | GPIO_AFRL_AFRL6_3);

	GPIOA->AFR[0] |= GPIO_AFRL_AFRL7_0 | GPIO_AFRL_AFRL7_2;
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL7_1 | GPIO_AFRL_AFRL7_3);

	// initialize SPI slave
	SPI1->CR1 |= SPI_CR1_DFF | SPI_CR1_RXONLY;			// 16-bit data frame, not using MISO
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST | SPI_CR1_SSM | 	// MSb first, disable software slave management
					SPI_CR1_SPE | SPI_CR1_MSTR | 		// disable SPI, slave mode
					SPI_CR1_BIDIMODE |					// not using bidirectional mode
					SPI_CR1_CPOL | SPI_CR1_CPHA);		// SPI mode = [0, 0]

	SPI1->CR2 &= ~(SPI_CR2_SSOE | SPI_CR2_FRF);			// disable slave select output and select Motorola mode
	SPI1->CR2 |= SPI_CR2_RXDMAEN;						// enable DMA requests when data is received


	/* DMA2 Channel 3, Stream 0 --> SPI1 RX */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;		// enable DMA2 clock

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;		// disable stream
	while (DMA2_Stream0->CR & DMA_SxCR_EN);	// wait for stream to disable

	DMA2_Stream0->PAR = (uint32_t)&(SPI1->DR);// peripheral address
	DMA2_Stream0->M0AR = (uint32_t)stream0;	// destination memory address (CT = 0)
	DMA2_Stream0->M1AR = (uint32_t)stream1;	// destination memory address (CT = 1)
	DMA2_Stream0->NDTR = N_BLOCK;			// number of units to be transmitted

	// select channel 3 for SPI1 RX
	DMA2_Stream0->CR &= ~DMA_SxCR_CHSEL;
	DMA2_Stream0->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1;

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

	SPI1->CR1 |= SPI_CR1_SPE;				// enable SPI1
}

void stream_splice(struct MicProc * mics)
{
	for (uint32_t i = 0; i < N_BLOCK; i += 3)
	{
		uint32_t ind = i / 3;

		// input stream looks like this
		// [CH0_S0, CH1_S0, CH2_S0, CH0_S1, CH1_S1, CH2_S1, ...]
		// it needs to be spliced like this
		// mic0 = [CH0_S0, CH0_S1, CH0_S2, ...]
		// mic1 = [CH1_S0, CH1_S1, CH1_S2, ...]
		// mic2 = [CH2_S0, CH2_S1, CH2_S2, ...]

		// DMA is targeting M1AR ---> read from M0AR memory
		if (dma_tgt)
		{
			for (uint32_t j = 0; j < N_MICS; j++)
				mics[j].raw[ind] = (float32_t) stream0[i + j] - 2048.0 * (1.0f / 2048.0f);		// convert raw ADC sample to range [-1, 1]
		}
		else
		{
			for (uint32_t j = 0; j < N_MICS; j++)
				mics[j].raw[ind] = (float32_t) stream1[i + j] - 2048.0 * (1.0f / 2048.0f);
		}
	}
}

uint8_t compute_event_pos(float32_t * x, float32_t * y, struct MicCoord * mics_xy, float32_t mic1_delay, float32_t mic2_delay)
{

	const float32_t max_step = 0.2f;			// maximum dx/dy change per iteration in meters

	// distances from mic1 and mic2 to mic0
	const float32_t d10 = SPEED_OF_SOUND * mic1_delay;
	const float32_t d20 = SPEED_OF_SOUND * mic2_delay;

	float32_t lambda = 1e-3f;

	float32_t old_res1 = 0, old_res2 = 0;

	for (uint8_t i = 0; i < 50; i++)
	{

		// compute radii of guesses
		float32_t r0 = sqrtf(powf(*x - mics_xy[0].x, 2) + powf(*y - mics_xy[0].y, 2));
		float32_t r1 = sqrtf(powf(*x - mics_xy[1].x, 2) + powf(*y - mics_xy[1].y, 2));
		float32_t r2 = sqrtf(powf(*x - mics_xy[2].x, 2) + powf(*y - mics_xy[2].y, 2));

		if (r0 == 0 || r1 == 0 || r2 == 0) return 1;

		// compute residuals (error)
		// [f]
		float32_t res1 = r1 - r0 - d10;
		float32_t res2 = r2 - r0 - d20;

		// compute cost, exit if small
		float32_t cost = res1 * res1 + res2 * res2;
		float32_t old_cost = old_res1 * old_res1 + old_res2 * old_res2;
		if (cost < 1e-8f)	return 1;

		// else, compare new residuals to ones calculated previous iteration
		if (cost < old_cost)		lambda *= 0.3f;			// reward improvement
		else						lambda *= 5.0f;			// punish bad step

		// create Jacobian
		float32_t j11 = (*x - mics_xy[1].x) / r1 - (*x - mics_xy[0].x) / r0;
		float32_t j12 = (*y - mics_xy[1].y) / r1 - (*y - mics_xy[0].y) / r0;
		float32_t j21 = (*x - mics_xy[2].x) / r2 - (*x - mics_xy[0].x) / r0;
		float32_t j22 = (*y - mics_xy[2].y) / r2 - (*y - mics_xy[0].y) / r0;

		// ([J]^T)[J] with damping on diagonal
		float32_t prod11 = j11 * j11 + j21 * j21 + lambda;
		float32_t prod12 = j11 * j12 + j21 * j22;
		float32_t prod21 = prod12;
		float32_t prod22 = j12 * j12 + j22 * j22 + lambda;

		// ([J]^T)[f]
		float32_t g1 = j11 * res1 + j21 * res2;
		float32_t g2 = j12 * res1 + j22 * res2;

		// solve system
		// ([J]^T)[J]delta = -([J]^T)[f]
		// delta = inv(([J]^T)[J]) * (-([J]^T)[f])
		float32_t det = prod11 * prod22 - prod12 * prod21;

		// ill-conditioned, increase lambda and try again
		if (fabsf(det) < 1e-12f)
		{
			lambda *= 10.0f;
			continue;
		}

		float32_t dx = (-prod22 * g1 + prod12 * g2) / det;
		float32_t dy = (prod21 * g1 - prod11 * g2) / det;

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

float32_t clamp(float32_t in, float32_t abs_max)
{
	// clamp in between [-abs_max, abs_max]
	if (in > abs_max)
		return abs_max;
	else if (in < -abs_max)
		return -abs_max;
	else
		return in;
}

void compute_envelope(struct MicProc * mics)
{
	// square signals and multiply with gain of 2
	for (uint32_t i = 0; i < N_MICS; i++)
		for (uint32_t j = 0; j < N_SAMPLE; j++)
			mics[i].envelope[j] = mics[i].samples[j] * mics[i].samples[j] * 2;

	// Lowpass filter to remove high frequencies accumulated during scaling
	for (uint32_t i = 0; i < N_MICS; i++)
		arm_fir_f32(&mics[i].lp_hfir, mics[i].envelope, mics[i].envelope, N_SAMPLE);

	// square root output
	for (uint32_t i = 0; i < N_MICS; i++)
		for (uint32_t j = 0; j < N_SAMPLE; j++)
			arm_sqrt_f32(mics[i].envelope[j], &mics[i].envelope[j]);
}

int32_t thresh_search(float32_t * src, uint32_t len, float32_t thresh)
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
