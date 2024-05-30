/*
 * rtc_main.h
 *
 *  Created on: Apr 20, 2024
 *      Author: ImadF
 */

#ifndef INC_RTC_MAIN_H_
#define INC_RTC_MAIN_H_

#include "stm32f4xx_hal.h"

#define SYS_CLOCK_FREQ_50_MHZ	 50
#define SYS_CLOCK_FREQ_84_MHZ 	 84
#define SYS_CLOCK_FREQ_120_MHZ 	 120

#define TRUE 	1
#define FALSE   0

#define SET		1
#define RESET	0

void Error_Handler();


#endif /* INC_RTC_MAIN_H_ */
