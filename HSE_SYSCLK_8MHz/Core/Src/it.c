/*
 * it.c
 *
 *  Created on: Apr 5, 2024
 *      Author: ImadF
 */
#include "stm32f4xx_hal.h"
#include "main_app.h"

extern UART_HandleTypeDef huart2;

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);

}
