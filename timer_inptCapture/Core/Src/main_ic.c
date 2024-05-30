/*
 * main_ic.c
 *
 *  Created on: Apr 9, 2024
 *      Author: ImadF
 */


#include "stm32f4xx_hal.h"
#include <string.h>
#include "main_ic.h"
#include <stdio.h>

void SystemClockConfig(uint32_t clock_freq);

void GPIO_Init(void);
void TIMER2_Init(void);
void TIMER6_Init(void);

void LSE_Config(void);
void UART2_Init(void);


uint32_t in_cap[2] = {0};
uint8_t count =1;
uint8_t is_captured = FALSE;

TIM_HandleTypeDef htimer2;
TIM_HandleTypeDef htimer6;

UART_HandleTypeDef huart2;

int main(void)
{
	char msg [100];
	uint32_t cap_diff = 0;
	double tim_cnt_freq = 0;
	double tim_per_res = 0;
	double signal_period = 0;
	double signal_freq = 0;

	HAL_Init();

	SystemClockConfig(SYS_CLOCK_FREQ_50MHZ);

	GPIO_Init();

	UART2_Init();

	TIMER6_Init();

	TIMER2_Init();

	//HAL_TIM_Base_Start_IT(&htimer2);

	LSE_Config();

    HAL_TIM_Base_Start_IT(&htimer6);

	HAL_TIM_IC_Start_IT(&htimer2, TIM_CHANNEL_1);

	while(1)
	{
		if(is_captured == TRUE)
		{
			if(in_cap[1] > in_cap[0])
			{
				cap_diff = in_cap[1] - in_cap[0];
			}
			else
			{
				cap_diff = 0xFFFFFFFF - in_cap[0] + in_cap[1];
			}

		tim_cnt_freq = (HAL_RCC_GetPCLK1Freq() *2) / (htimer2.Init.Prescaler+1);
		tim_per_res = 1/tim_cnt_freq;
		signal_period = cap_diff*tim_per_res;
		signal_freq = 1/signal_period;

		sprintf(msg, " Applied signal frequency is : %lf \r \n", signal_freq);

		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		is_captured = FALSE;

		}
   }

	return 0;
}

void SystemClockConfig(uint32_t clock_freq)
{
	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;

				//for HSE
	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
	osc_init.HSIState = RCC_HSI_ON; // FOR DISACOVERY
	osc_init.LSEState = RCC_LSE_ON; // FOR DISACOVERY
	osc_init.HSICalibrationValue = 16;
	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	osc_init.PLL.PLLState = RCC_PLL_ON;

	uint32_t FLatency = 0;
	switch(clock_freq)
	{
		case SYS_CLOCK_FREQ_50MHZ:
		{
			//osc_init.PLL.PLLM = 8; for hse

			osc_init.PLL.PLLM = 16;
			osc_init.PLL.PLLN = 100;
			osc_init.PLL.PLLP = 2;
			osc_init.PLL.PLLQ = 2;

			clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_SYSCLK;
			clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
			clk_init.APB1CLKDivider = RCC_SYSCLK_DIV2;
			clk_init.APB2CLKDivider = RCC_SYSCLK_DIV2;
			clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

			FLatency = FLASH_ACR_LATENCY_1WS;


			break;

		}
		case SYS_CLOCK_FREQ_84MHZ:
		{
			//osc_init.PLL.PLLM = 8; for hse

			osc_init.PLL.PLLM = 16;
			osc_init.PLL.PLLN = 168;
			osc_init.PLL.PLLP = 2;
			osc_init.PLL.PLLQ = 2;

			clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_SYSCLK;
			clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
			clk_init.APB1CLKDivider = RCC_SYSCLK_DIV2;
			clk_init.APB2CLKDivider = RCC_SYSCLK_DIV2;
			clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			FLatency = FLASH_ACR_LATENCY_2WS;

			break;
		}
		case SYS_CLOCK_FREQ_120MHZ:
		{
			//osc_init.PLL.PLLM = 8; for hse

			osc_init.PLL.PLLM = 16;
			osc_init.PLL.PLLN = 240;
			osc_init.PLL.PLLP = 2;
			osc_init.PLL.PLLQ = 2;

			clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_SYSCLK;
			clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
			clk_init.APB1CLKDivider = RCC_SYSCLK_DIV4;
			clk_init.APB2CLKDivider = RCC_SYSCLK_DIV2;
			clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			FLatency = FLASH_ACR_LATENCY_3WS;

			break;
		}

		case SYS_CLOCK_FREQ_168MHZ:
		{
			//enable clock for pwr controller
			__HAL_RCC_PWR_CLK_ENABLE();

			//set vo scale as 1
			__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

			osc_init.PLL.PLLM = 8; //for hse
			osc_init.PLL.PLLN = 336;
			osc_init.PLL.PLLP = 2;
			osc_init.PLL.PLLQ = 2;

			clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_SYSCLK;
			clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
			clk_init.APB1CLKDivider = RCC_SYSCLK_DIV4;
			clk_init.APB2CLKDivider = RCC_SYSCLK_DIV2;
			clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			FLatency = FLASH_ACR_LATENCY_5WS;

			break;
		}

		default:
			return;
	}


	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_RCC_ClockConfig(&clk_init, FLatency) != HAL_OK)
	{
		Error_Handler();
	}

}


void Error_Handler()
{
	while(1);
}

void GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_LED;
	GPIO_LED.Pin = GPIO_PIN_12;
	GPIO_LED.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_LED.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_LED);
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

	if( HAL_UART_Init(&huart2) != HAL_OK)
	{
		//error
		Error_Handler();
	}
}


void TIMER2_Init(void)
{
	htimer2.Instance = TIM2;
	htimer2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htimer2.Init.Period = 0xFFFFFFFF;
	htimer2.Init.Prescaler = 1;

	if(HAL_TIM_IC_Init(&htimer2) != HAL_OK)
	{
		Error_Handler();
	}

	TIM_IC_InitTypeDef TIM_IC_Config;
	TIM_IC_Config.ICFilter = 0;
	TIM_IC_Config.ICPolarity = TIM_ICPOLARITY_RISING;
	TIM_IC_Config.ICPrescaler = TIM_ICPSC_DIV1;
	TIM_IC_Config.ICSelection = TIM_ICSELECTION_DIRECTTI;

	if(HAL_TIM_IC_ConfigChannel(&htimer2, &TIM_IC_Config, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
}

void TIMER6_Init(void)
{
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 9;
	htimer6.Init.Period = 50-1;
	if( HAL_TIM_Base_Init(&htimer6) != HAL_OK )
	{
		Error_Handler();
	}

}


void LSE_Config(void)
{
#if 0
	RCC_OscInitTypeDef osc_init;

					//for HSE
	osc_init.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	osc_init.HSEState = RCC_LSE_ON;// FOR DISACOVERY

	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK)
	{
		Error_Handler();
	}
#endif
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);

}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(! is_captured)
	{

		if(count == 1)
		{
			//in_cap[0] = __HAL_TIM_GET_COMPARE(htimer2, TIM_CHANNEL_1);
			in_cap[0] = HAL_TIM_ReadCapturedValue(&htimer2, TIM_CHANNEL_1);
			count++;
		}

		else if(count == 2)
		{
			//in_cap[1] = __HAL_TIM_GET_COMPARE(htimer2, TIM_CHANNEL_1);
			in_cap[1] = HAL_TIM_ReadCapturedValue(&htimer2, TIM_CHANNEL_1);

			count = 1;
			is_captured = TRUE;
		}
	}
}
