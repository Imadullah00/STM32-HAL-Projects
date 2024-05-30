/*
 * CAN_N_msp.c
 *
 *  Created on: Apr 16, 2024
 *      Author: ImadF
 */

#include "stm32f4xx_hal.h"
#include "standby_main.h"

extern TIM_HandleTypeDef htimer2;

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

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	 //configure the low level inits

	 //1. enable the clock for the usart2
	 __HAL_RCC_USART2_CLK_ENABLE();
	 __HAL_RCC_USART2_CLK_SLEEP_DISABLE();

	 __HAL_RCC_GPIOA_CLK_ENABLE();
	 //2. do the pin muxing config.
	 GPIO_InitTypeDef gpio_uart;

	 gpio_uart.Pin = GPIO_PIN_2;	//gpio uart tx
	 gpio_uart.Mode = GPIO_MODE_AF_PP;
	 gpio_uart.Pull = GPIO_PULLUP;
	 gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;
	 gpio_uart.Alternate = GPIO_AF7_USART2;

	 HAL_GPIO_Init(GPIOA, &gpio_uart);

	 gpio_uart.Pin = GPIO_PIN_3; 	//gpio uart Rx

	 HAL_GPIO_Init(GPIOA, &gpio_uart);


	 //3, enable the irq and set up the priority (NVIC settings) (to be used if polling mode isnt used)
	 HAL_NVIC_EnableIRQ(USART2_IRQn);
	 HAL_NVIC_SetPriority(USART2_IRQn, 15, 0);
}


