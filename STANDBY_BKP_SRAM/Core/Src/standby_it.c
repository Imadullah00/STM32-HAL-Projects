/*
 * CAN_N_it.c
 *
 *  Created on: Apr 16, 2024
 *      Author: ImadF
 */
#include "stm32f4xx_hal.h"

extern  TIM_HandleTypeDef htimer6;
extern UART_HandleTypeDef huart2;

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


/*void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}


void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

*/
