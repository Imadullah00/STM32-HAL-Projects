/*
 * WFI_main.c
 *
 *  Created on: Apr 19, 2024
 *      Author: ImadF
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <WFE_main.h>

void SystemClock_Config_HSE(uint8_t clock_freq);
void Error_Handler();
void GPIO_Init(void);
void UART2_Init();
void GPIO_Analog_Config();

UART_HandleTypeDef huart2;

extern uint8_t some_data [] ;

int main(void)
{
	char msg[50];

	HAL_Init();

	//SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);  //using HSI

	GPIO_Init();

	UART2_Init();

	GPIO_Analog_Config();
	//HAL_PWR_EnableSleepOnExit();

	while(1)
	{

	  if(HAL_UART_Transmit(&huart2, (uint8_t*) some_data, strlen((char*) some_data), HAL_MAX_DELAY) != HAL_OK)
	  	{
	  		Error_Handler();
	  	}

	  memset(msg,0,sizeof(msg));
	  sprintf(msg, "Going to sleep \r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen((char*) msg), HAL_MAX_DELAY);

	  HAL_SuspendTick(); //Disble systick otherwise the Event Reg will always be set due to
	  	  	  	  	  	 // Systick Interrupts

	  __SEV();

	  __WFE();  //go to sleep after executing this

	  __WFE();  //go to sleep after executing this

	   //resume here on WAKEUP

	  HAL_ResumeTick(); //enable the Systick
	  memset(msg,0,sizeof(msg));


	  sprintf(msg, "Woke up! \r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen((char*) msg), HAL_MAX_DELAY) ;

	}
	return 0;
}

void SystemClock_Config_HSE(uint8_t clock_freq)
{

	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;
	uint8_t flash_latency = 0;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
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
void Error_Handler()
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

void GPIO_Analog_Config()
{
	GPIO_InitTypeDef gpioa_an;

	gpioa_an.Pin =  GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
				   GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
				   GPIO_PIN_14 | GPIO_PIN_15;

	gpioa_an.Mode = GPIO_MODE_ANALOG;

	HAL_GPIO_Init(GPIOA, &gpioa_an);
}

void GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//__HAL_RCC_GPIOD_CLK_ENABLE();

	//for input user button in first board
	GPIO_InitTypeDef GPIO_BTN;
	GPIO_BTN.Pin = GPIO_PIN_0;
	GPIO_BTN.Mode = GPIO_MODE_EVT_FALLING; // instead of an int, an event will be generated and
										   // event reg will be set. For falling edge.
	GPIO_BTN.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &GPIO_BTN);

	//HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0); //  commented b/c no interrupt
	//HAL_NVIC_EnableIRQ(EXTI0_IRQn); //commented to disable interrupts in order to execute WFE

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// UART  code shifted to main because this callback won't be called now
	// as the inrpt wont be generated
}
