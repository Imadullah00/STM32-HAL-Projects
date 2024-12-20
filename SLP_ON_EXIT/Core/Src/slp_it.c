/*
 * slp_it.c
 *
 *  Created on: Apr 18, 2024
 *      Author: ImadF
 */

#include "stm32f4xx_hal.h"

extern  TIM_HandleTypeDef htimer6;


void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM6_DAC_IRQHandler(void)
{
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
	HAL_TIM_IRQHandler(&htimer6);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);

}
