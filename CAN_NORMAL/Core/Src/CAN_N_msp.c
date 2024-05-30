/*
 * CAN_N_msp.c
 *
 *  Created on: Apr 16, 2024
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

 void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
 {
 	__HAL_RCC_CAN1_CLK_ENABLE();

 	//PD0-->CANRX
 	//PD1--->CANTX

 	GPIO_InitTypeDef gpiod;

 	gpiod.Mode = GPIO_MODE_AF_PP;
 	gpiod.Pin = GPIO_PIN_0 | GPIO_PIN_1;
 	gpiod.Pull = GPIO_NOPULL;
 	gpiod.Speed = GPIO_SPEED_HIGH;
 	gpiod.Alternate = GPIO_AF9_CAN1;

 	HAL_GPIO_Init(GPIOD, &gpiod);

 	//SET PRIORITY FOR DIFFERENT INTERRUPTS
 	HAL_NVIC_SetPriority(CAN1_TX_IRQn, 15, 0);
 	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 15, 0);
 	HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 15, 0);
 	HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 15, 0);

 	//Enable the IRQs
 	HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
 	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
 	HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
 	HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}


 void HAL_UART_MspInit(UART_HandleTypeDef *huart)
 {
 	 GPIO_InitTypeDef gpio_uart;
 	 //here we are going to do the low level inits. of the USART2 peripheral

 	 //1. enable the clock for the USART2 peripheral as well as for GPIOA peripheral
 	 __HAL_RCC_USART2_CLK_ENABLE();
 	 __HAL_RCC_GPIOA_CLK_ENABLE();

 	 //2 . Do the pin muxing configurations
 	 gpio_uart.Pin = GPIO_PIN_2;
 	 gpio_uart.Mode =GPIO_MODE_AF_PP;
 	 gpio_uart.Pull = GPIO_PULLUP;
 	 gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;
 	 gpio_uart.Alternate =  GPIO_AF7_USART2; //UART2_TX
 	 HAL_GPIO_Init(GPIOA,&gpio_uart);

 	 gpio_uart.Pin = GPIO_PIN_3; //UART2_RX
 	 HAL_GPIO_Init(GPIOA,&gpio_uart);

 	 //3 . Enable the IRQ and set up the priority (NVIC settings )
 	 HAL_NVIC_EnableIRQ(USART2_IRQn);
 	 HAL_NVIC_SetPriority(USART2_IRQn,15,0);

 }
