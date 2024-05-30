/*
 * CAN_N_it.c
 *
 *  Created on: Apr 16, 2024
 *      Author: ImadF
 */
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hCAN1;
extern TIM_HandleTypeDef htimer6;


void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void TIM6_DAC_IRQHandler(void)
{
	TIM6->SR = 0;

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


void EXTI0_IRQHandler(void)
{
	HAL_TIM_Base_Start_IT(&htimer6);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
