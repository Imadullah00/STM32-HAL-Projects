/*
 * CAN_it.c
 *
 *  Created on: Apr 14, 2024
 *      Author: ImadF
 */

#include "stm32f4xx_hal.h"
#include "CAN_main.h"

extern CAN_HandleTypeDef hCAN1;

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void CAN1_TX_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hCAN1);
}
void CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hCAN1);

}
void CAN1_RX1_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hCAN1);

}
void CAN1_SCE_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hCAN1);

}
