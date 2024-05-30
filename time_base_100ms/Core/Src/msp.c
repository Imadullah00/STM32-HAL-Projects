/*
 * msp.c
 *
 *  Created on: Apr 5, 2024
 *      Author: ImadF
 */

#include "stm32f4xx_hal.h"

 void HAL_MspInit(void)
{
	 // Processor specific low level inits

	 //1. Set up the priority grouping of the arm cortex m4 processor
	 HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	 //2. Enable the system exceptions
	 SCB->SHCSR |= 0x7 << 16;

	 //3. Set up the priority for the system exceptions.
	 HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0); //for mem manage fault
	 HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0); //for bus fault
	 HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0); //for usage fault

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	//1. enable the TIM6 clock
	__HAL_RCC_TIM6_CLK_ENABLE();

	//2. Enable the interrupt
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 15, 0);
}
