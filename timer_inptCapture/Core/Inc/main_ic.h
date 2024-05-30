/*
 * main_ic.h
 *
 *  Created on: Apr 9, 2024
 *      Author: ImadF
 */

#ifndef INC_MAIN_IC_H_
#define INC_MAIN_IC_H_

#include "stm32f4xx_hal.h"

#define SYS_CLOCK_FREQ_50MHZ	50
#define SYS_CLOCK_FREQ_84MHZ	84
#define SYS_CLOCK_FREQ_120MHZ	120
#define SYS_CLOCK_FREQ_168MHZ	168

void Error_Handler();

#define TRUE	1
#define FALSE	0

#endif /* INC_MAIN_IC_H_ */
