/*
 * app_callback.c
 *
 *  Created on: Nov 19, 2024
 *      Author: ImadF
 */
#include "main.h"

extern uint32_t g_counter;
extern volatile uint32_t g_ch1_state; //green
extern volatile uint32_t g_ch2_state; //red



#if 0
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

	g_counter = 0;
	g_ch1_state = 8000;
	g_ch2_state = 4000;

}
#endif

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) //called once pwm pulse goes low
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 ){
		g_ch1_state = 0; }
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 ){
		g_ch2_state = 0; }

}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{

}
