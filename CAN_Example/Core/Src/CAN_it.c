/*
 * CAN_it.c
 *
 *  Created on: Apr 14, 2024
 *      Author: ImadF
 */

#include "CAN_main.h"


void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
