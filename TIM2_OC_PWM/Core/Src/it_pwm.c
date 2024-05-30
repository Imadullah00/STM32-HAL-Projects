/*
 * it_pwm.c
 *
 *  Created on: Apr 12, 2024
 *      Author: ImadF
 */
#include "main_pwm.h"

extern TIM_HandleTypeDef htimer2;

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer2);
}

