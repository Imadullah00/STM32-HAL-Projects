/*
 * main_oc.c
 *
 *  Created on: Apr 11, 2024
 *      Author: ImadF
 */
#include "main_oc.h"
#include "stm32f4xx_hal.h"
#include "string.h"

void GPIO_Init(void);
void Error_Handler(void);
void UART2_Init();
void TIM4_Init(void);
void SystemClock_Config_HSE(uint8_t clock_freq);


UART_HandleTypeDef huart2;
TIM_HandleTypeDef htimer4;

volatile uint32_t pulse_1hz = 2500;
volatile uint32_t pulse_500hz = 25000;
volatile uint32_t pulse_1khz = 12500;
volatile uint32_t pulse_2khz = 6250;
volatile uint32_t pulse_4khz = 3125;

volatile uint32_t ccr_content = 0;

int main(void)
{

	HAL_Init();

	SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);

	GPIO_Init();

	UART2_Init();

	TIM4_Init();

	if( HAL_TIM_OC_Start_IT(&htimer4,TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	/*if( HAL_TIM_OC_Start_IT(&htimer4,TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

	if( HAL_TIM_OC_Start_IT(&htimer4,TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}

	if( HAL_TIM_OC_Start_IT(&htimer4,TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}*/


	while(1);

	return 0;
}

void SystemClock_Config_HSE(uint8_t clock_freq)
{

	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;
    uint8_t flash_latency=0;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE ;
	Osc_Init.HSEState = RCC_HSE_ON;
	//Osc_Init.LSEState = RCC_LSE_ON;
	//Osc_Init.HSIState = RCC_HSI_ON;
	Osc_Init.PLL.PLLState = RCC_PLL_ON;
	Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	switch(clock_freq)
	 {
	  case SYS_CLOCK_FREQ_50_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 50;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
          flash_latency = 1;
	     break;

	  case SYS_CLOCK_FREQ_84_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 84;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
          flash_latency = 2;
	     break;

	  case SYS_CLOCK_FREQ_120_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 120;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV4;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV2;
          flash_latency = 3;
	     break;

	  default:
	   return ;
	 }

		if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK)
	{
			Error_Handler();
	}



	if (HAL_RCC_ClockConfig(&Clock_Init, flash_latency) != HAL_OK)
	{
		Error_Handler();
	}


	/*Configure the systick timer interrupt frequency (for every 1 ms) */
	uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
	HAL_SYSTICK_Config(hclk_freq/1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

 }

void Error_Handler(void)
{
	while(1);
}


void UART2_Init()
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if ( HAL_UART_Init(&huart2) != HAL_OK )
	{
		//There is a problem
		Error_Handler();
	}

}


void TIM4_Init(void)
{
	htimer4.Instance = TIM4;
	htimer4.Init.Period = 0xFFFFFFFF;
	htimer4.Init.Prescaler = 5000-1;

	if(HAL_TIM_OC_Init(&htimer4) != HAL_OK)
	{
		Error_Handler();
	}

	TIM_OC_InitTypeDef tim4OC_Init;

	tim4OC_Init.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim4OC_Init.OCMode = TIM_OCMODE_TOGGLE;
//	tim4OC_Init.Pulse = pulse_500hz;
	tim4OC_Init.Pulse = pulse_1hz;


	if(HAL_TIM_OC_ConfigChannel(&htimer4, &tim4OC_Init, TIM_CHANNEL_1)!= HAL_OK)
		{
			Error_Handler();
		}

	/*tim4OC_Init.Pulse = pulse_1khz;

	if(HAL_TIM_OC_ConfigChannel(&htimer4, &tim4OC_Init, TIM_CHANNEL_2)!= HAL_OK)
		{
			Error_Handler();
		}

	tim4OC_Init.Pulse = pulse_2khz;

	if(HAL_TIM_OC_ConfigChannel(&htimer4, &tim4OC_Init, TIM_CHANNEL_3)!= HAL_OK)
		{
			Error_Handler();
		}

	tim4OC_Init.Pulse = pulse_4khz;

	if(HAL_TIM_OC_ConfigChannel(&htimer4, &tim4OC_Init, TIM_CHANNEL_4)!= HAL_OK)
		{
			Error_Handler();
		} */

}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
 {
   /* TIM3_CH1 toggling with frequency = 500 Hz */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
   {
	   ccr_content = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
	   __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,ccr_content+pulse_1hz);
   }

   /* TIM3_CH2 toggling with frequency = 1000 Hz */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
   {
	   ccr_content = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
	   __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,ccr_content+pulse_1khz);

   }

   /* TIM3_CH3 toggling with frequency = 2000 Hz */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
   {
	   ccr_content = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
	   __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3,ccr_content+pulse_2khz);

   }

   /* TIM3_CH4 toggling with frequency = 4000 Hz */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
   {
	    ccr_content = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
	   __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,ccr_content+pulse_4khz);

   }
 }

void GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_12;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);
}


