/*
 * it.c
 *
 *  Created on: Apr 5, 2024
 *      Author: ImadF
 */
#include "stm32f4xx_hal.h"
#include "main_app.h"

extern  TIM_HandleTypeDef htimer6;

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer6);

}
