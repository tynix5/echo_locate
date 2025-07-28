/* USER CODE BEGIN Header */
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

#define NUM_FILTER_TAPS		59
#define BLOCK_SIZE			32

float32_t const ftaps[NUM_FILTER_TAPS] = {
		 0.008853489517862969,	0.012336117846197307,	0.01919053080081171,	0.02357529588293749,	0.02103778430880732,
		 0.009851119935668212,	-0.006868821885649766,	-0.022287674656347622, 	-0.029593270905834943,	-0.025971486257862348,
		 -0.014458722112174098,	-0.002576204552428596,	0.0015612559742238874,	-0.006084381509796098,	-0.02270617511778867,
		 -0.03946159315255754,	-0.046166772454251284,	-0.03768506203086776,	-0.018208106399573615,	-0.00024668883022941945,
		 0.0020573263647973195,	-0.017953408395298347,	-0.05326257976724875,	-0.08428105251091034,	-0.08766353278952717,
		 -0.048803421864686024,	0.028667251505804254,	0.12188932879853132,	0.1975603429597411,		0.22660949366725555,
		 0.1975603429597411,	0.12188932879853132,	0.028667251505804254,	-0.048803421864686024,	-0.08766353278952717,
		 -0.08428105251091034,	-0.05326257976724875,	-0.017953408395298347,	0.0020573263647973195,	-0.00024668883022941945,
		 -0.018208106399573615,	-0.03768506203086776,	-0.046166772454251284,	-0.03946159315255754,	-0.02270617511778867,
		 -0.006084381509796098,	0.0015612559742238874,	-0.002576204552428596,	-0.014458722112174098,	-0.025971486257862348,
		 -0.029593270905834943,	-0.022287674656347622,	-0.006868821885649766,	0.009851119935668212,	0.02103778430880732,
		 0.02357529588293749,	0.01919053080081171,	0.012336117846197307,	0.008853489517862969
};

arm_fir_instance_f32 hfir;

uint16_t stream[BLOCK_SIZE];
float stream_f[BLOCK_SIZE];


void sysclock_config(void);
void adc1_dma_config(void);
void tim2_trig_config(void);


int main(void)
{

	sysclock_config();

	float32_t state[NUM_FILTER_TAPS + BLOCK_SIZE - 1];
	arm_fir_init_f32(&hfir, NUM_FILTER_TAPS, ftaps, state, BLOCK_SIZE);

	float32_t out_buff[BLOCK_SIZE];

	uart2_set_fcpu(84000000);
	uart2_config(115200, USART_DATA_8, USART_STOP_1);

	tim2_trig_config();
	adc1_dma_config();

	while (1)
	{

		while (!((DMA2->LISR) & DMA_LISR_TCIF0));		// wait for stream to complete
		ADC1->CR2 &= ~ADC_CR2_DMA;						// disable ADC DMA
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;			// clear transfer complete and half complete flag

		for (uint8_t i = 0; i < BLOCK_SIZE; i++)
		{
			stream_f[i] = (float) stream[i];
		}

		arm_fir_f32(&hfir, stream_f, out_buff, BLOCK_SIZE);

		ADC1->CR2 |= ADC_CR2_DMA;


		uint8_t str[11];

		for (uint32_t i = 0; i < BLOCK_SIZE; i++)
		{
			str[0] = (uint16_t) stream[i] / 1000 + '0';
			str[1] = (uint16_t) stream[i] / 100 % 10 + '0';
			str[2] = (uint16_t) stream[i] / 10 % 10 + '0';
			str[3] = (uint16_t) stream[i] % 10 + '0';
			str[4] = ',';
			str[5] = ' ';
			str[6] = (uint16_t) out_buff[i] / 1000 + '0';
			str[7] = (uint16_t) out_buff[i] / 100 % 10 + '0';
			str[8] = (uint16_t) out_buff[i] / 10 % 10 + '0';
			str[9] = (uint16_t) out_buff[i] % 10 + '0';
			str[10] = '\r';
			uart2_write(str, 11);
		}
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

	ADC1->SQR1 &= ~ADC_SQR1_L;				// 1 conversion per sequence
	ADC1->SQR3 &= ~ADC_SQR3_SQ1;			// channel 0 (PA0) is conversion channel

	/* DMA2 Channel 0, Stream 0 --> ADC1 */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;		// enable DMA2 clock

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;		// disable stream
	while (DMA2_Stream0->CR & DMA_SxCR_EN);	// wait for stream to disable

	DMA2_Stream0->PAR = (uint32_t)&(ADC1->DR);// peripheral address
	DMA2_Stream0->M0AR = (uint32_t)stream;	// destination memory address
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


	DMA2_Stream0->CR |= DMA_SxCR_EN;		// enable DMA stream

	ADC1->CR2 |= ADC_CR2_DMA;				// enable DMA
	ADC1->CR2 &= ~ADC_CR2_DDS;				// no DMA requests after last transfer (BLOCK_SIZE transfers)
	ADC1->CR2 |= ADC_CR2_ADON;				// turn on ADC
}


void tim2_trig_config(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;				// enable TIM2 clock

	TIM2->CR1 &= ~TIM_CR1_DIR;			// upcounting
	TIM2->PSC = 0;						// /1 prescaler
	TIM2->ARR = 2100;					// 40kHz frequency

	TIM2->CR2 |= TIM_CR2_MMS_1;
	TIM2->CR2 &= ~(TIM_CR2_MMS_0 | TIM_CR2_MMS_2);		// update event as TRGO

	TIM2->EGR |= TIM_EGR_UG;			// generate update event

	TIM2->CR1 |= TIM_CR1_CEN;			// enable counter
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
