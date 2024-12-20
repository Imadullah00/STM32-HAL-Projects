#include "main_app.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include<stdint.h>
#include <stdio.h>

#define FALSE	0
#define TRUE	1

void UART2_Init();
void Error_Handler();

UART_HandleTypeDef huart2;

uint8_t rcvd_it;
uint8_t data_buffer [100];
uint8_t RxComplt = FALSE;
uint16_t count = 0;

char msg [100];

char *user_data = " The application is running\r\n";

int main(void)
{
	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;

	memset(&osc_init, 0, sizeof(osc_init));
	memset(&clk_init, 0, sizeof(clk_init));

	HAL_Init();

	UART2_Init(); //redundant

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc_init.HSEState = RCC_HSE_ON;  //applicable only for discovery board
	if( HAL_RCC_OscConfig(&osc_init) != HAL_OK)
	{
		Error_Handler();
	}

	clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_SYSCLK;
	clk_init.AHBCLKDivider = RCC_SYSCLK_DIV2;
	clk_init.APB1CLKDivider = RCC_SYSCLK_DIV2;
	clk_init.APB2CLKDivider = RCC_SYSCLK_DIV2;
	clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	if(HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}

	UART2_Init();

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	memset(msg, 0,  sizeof(msg));
	sprintf(msg, "SYSCLK: %ld\r\n", HAL_RCC_GetSysClockFreq());
	HAL_UART_Transmit(&huart2,(uint8_t*) msg, sizeof(msg), HAL_MAX_DELAY);

	memset(msg, 0,  sizeof(msg));
	sprintf(msg, "HCLK: %ld\r\n", HAL_RCC_GetHCLKFreq());
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, sizeof(msg), HAL_MAX_DELAY);

	memset(msg, 0,  sizeof(msg));
	sprintf(msg, "PCLK1: %ld\r\n", HAL_RCC_GetPCLK1Freq());
	HAL_UART_Transmit(&huart2,(uint8_t*) msg, sizeof(msg), HAL_MAX_DELAY);

	memset(msg, 0,  sizeof(msg));
	sprintf(msg, "PCLK2: %ld\r\n", HAL_RCC_GetPCLK2Freq());
	HAL_UART_Transmit(&huart2,(uint8_t*) msg, sizeof(msg), HAL_MAX_DELAY);

	while(1);

	return 0;
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
void Error_Handler()
{
	while(1);
}
