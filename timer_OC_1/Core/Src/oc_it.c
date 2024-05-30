/*
 * oc_it.c
 *
 *  Created on: Apr 11, 2024
 *      Author: ImadF
 */

#include "main_oc.h"

extern TIM_HandleTypeDef htimer4;

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void TIM4_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer4);
}


