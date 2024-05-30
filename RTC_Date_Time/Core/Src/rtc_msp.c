/*
 * rtc_msp.c
 *
 *  Created on: Apr 20, 2024
 *      Author: ImadF
 */


#include "stm32f4xx_hal.h"
#include "rtc_main.h"

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

void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
	RCC_OscInitTypeDef osc_init;

	osc_init.HSEState = RCC_HSE_ON;
	//osc_init.HSIState = RCC_HSI_OFF;
	//osc_init.LSEState = RCC_LSE_OFF;
	//osc_init.LSIState = RCC_LSI_OFF;
	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc_init.PLL.PLLState = RCC_PLL_NONE;

	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK)
	{
		Error_Handler();
	}

	//2. select HSE as RTC clock
	RCC_PeriphCLKInitTypeDef peri_clk;

	peri_clk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	peri_clk.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV8;

	if(HAL_RCCEx_PeriphCLKConfig(&peri_clk) != HAL_OK)
	{
		Error_Handler();
	}

	//3. enable the rtc clock
	__HAL_RCC_RTC_ENABLE();
}


