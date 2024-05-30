/*
 * CAN_N_main.h
 *
 *  Created on: Apr 16, 2024
 *      Author: ImadF
 */

#ifndef INC_CAN_N_MAIN_H_
#define INC_CAN_N_MAIN_H_

#include "stm32f4xx_hal.h"

#define SYS_CLOCK_FREQ_50_MHZ	 50
#define SYS_CLOCK_FREQ_84_MHZ 	 84
#define SYS_CLOCK_FREQ_120_MHZ 	 120

#define TRUE 	1
#define FALSE   0

#define SET		1
#define RESET	0

#define LED_GREEN	GPIO_PIN_12
#define LED_ORANGE	GPIO_PIN_13
#define LED_RED		GPIO_PIN_14
#define LED_BLUE	GPIO_PIN_15

#endif /* INC_CAN_N_MAIN_H_ */
