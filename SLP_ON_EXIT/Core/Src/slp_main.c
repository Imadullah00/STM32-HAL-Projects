/*
 * slp_main.c
 *
 *  Created on: Apr 18, 2024
 *      Author: ImadF
 */

#include "slp_main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

void SystemClock_Config_HSE(uint8_t clock_freq);
void Error_Handler();
void TIMER_Init(void);
void GPIO_Init(void);
void UART2_Init();
void GPIO_Analog_Config();
extern uint8_t some_data ;

TIM_HandleTypeDef htimer6;
UART_HandleTypeDef huart2;

int main(void)
{
	HAL_Init();

	SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);

	GPIO_Init();

	TIMER_Init();

	GPIO_Analog_Config();

	HAL_PWR_EnableSleepOnExit();

	TIM6->SR = 0; //reset the SR to prevent spurious interrupts

	HAL_TIM_Base_Start_IT(&htimer6);

	while(1);

	return 0;
}

void SystemClock_Config_HSE(uint8_t clock_freq)
{

	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;
	uint8_t flash_latency = 0;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_HSI ;
	Osc_Init.HSEState = RCC_HSE_ON;
	Osc_Init.LSEState = RCC_LSE_ON;
	Osc_Init.HSIState = RCC_HSI_ON;
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
void Error_Handler()
{
	while(1);
}

void GPIO_Init(void)
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_LED;
	GPIO_LED.Pin = GPIO_PIN_12;
	GPIO_LED.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_LED.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_LED);

	GPIO_InitTypeDef GPIO_DBG;
	GPIO_DBG.Pin = GPIO_PIN_11;
	GPIO_DBG.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_DBG.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_DBG);
}


void TIMER_Init(void)
{
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 4999;
	htimer6.Init.Period = 100-1;

	if(HAL_TIM_Base_Init(&htimer6) != HAL_OK)
	{
		Error_Handler();
	}
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(HAL_UART_Transmit(&huart2, (uint8_t*) some_data, (uint16_t) strlen((char*) some_data), HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
}

void GPIO_Analog_Config()
{
	GPIO_InitTypeDef gpioa_an;

	gpioa_an.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
				   GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
				   GPIO_PIN_14 | GPIO_PIN_15;

	gpioa_an.Mode = GPIO_MODE_ANALOG;

	HAL_GPIO_Init(GPIOA, &gpioa_an);
}
