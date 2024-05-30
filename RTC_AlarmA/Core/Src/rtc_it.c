/*
 * rtc_it.c
 *
 *  Created on: Apr 20, 2024
 *      Author: ImadF
 */


#include "stm32f4xx_hal.h"
#include "rtc_main.h"

extern UART_HandleTypeDef huart2;
extern RTC_HandleTypeDef hRTC;


void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void EXTI0_IRQHandler(void)
{

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}


void RTC_Alarm_IRQHandler(void)
{
	HAL_RTC_AlarmIRQHandler(&hRTC);
}
