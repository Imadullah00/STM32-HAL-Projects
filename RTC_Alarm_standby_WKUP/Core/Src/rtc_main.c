/*
 * rtc_main.c
 *
 *  Created on: Apr 20, 2024
 *      Author: ImadF
 */

#include <rtc_main.h>
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <string.h>
#include<stdint.h>
#include<stdarg.h>

extern char some_data [];

void SystemClock_Config_HSE(uint8_t clock_freq);
void GPIO_Init(void);
void UART2_Init();
void GPIO_Analog_Config();
void RTC_Init();
void RTC_Calendar_Config();
char* getDayofWeek(uint8_t number);
char* getAM_PM(uint8_t number);
void RTC_Alarm_Config();

UART_HandleTypeDef huart2;
RTC_HandleTypeDef hRTC;

void printmsg(char *format,...)
{
	char str[80];
	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format,args);
	HAL_UART_Transmit(&huart2,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
	va_end(args);
}

int main(void)
{
	HAL_Init();

	SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);

	GPIO_Init();

	UART2_Init();

	RTC_Init();

	printmsg("This is RTC Calendar Testing program\r\n");

	__HAL_RCC_PWR_CLK_ENABLE();

	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	{
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		__HAL_RTC_ALARM_CLEAR_FLAG(&hRTC, RTC_FLAG_ALRAF);

		printmsg("woke up from the standby mode\r\n");


		RTC_TimeTypeDef get_time;

		HAL_RTC_GetTime(&hRTC, &get_time, RTC_FORMAT_BIN);


		printmsg("Current Time is %02d:%02d:%02d\r\n", get_time.Hours, get_time.Minutes, get_time.Seconds );

		RTC_DateTypeDef get_date;

		HAL_RTC_GetDate(&hRTC, &get_date, RTC_FORMAT_BIN);

		printmsg("Current Date is: %02d-%02d-%02d<%s>  <%s> \r\n" , get_date.Month, get_date.Date, get_date.Year, getAM_PM(get_date.WeekDay), getDayofWeek(get_date.WeekDay));

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

		HAL_Delay(2000); //malfunctions if current isr's priority is higher than systick. Because delay function depends on systick

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

	}



	//Enable the wakeup pin 1 in pwr_csr register
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);



	while(1);

	return 0;
}


void SystemClock_Config_HSE(uint8_t clock_freq)
{

	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;
	uint8_t flash_latency = 0;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI ;
	Osc_Init.HSEState = RCC_HSE_ON;
	Osc_Init.LSEState = RCC_LSI_ON;
//	Osc_Init.HSIState = RCC_HSI_ON;
	Osc_Init.PLL.PLLState = RCC_PLL_ON;
	Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	switch(clock_freq)
	{
	  case SYS_CLOCK_FREQ_50_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 50;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
		  flash_latency = 1;
		 break;

	  case SYS_CLOCK_FREQ_84_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 84;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
		  flash_latency = 2;
		 break;

	  case SYS_CLOCK_FREQ_120_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 120;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV4;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV2;
		  flash_latency = 3;
		 break;

	  default:
	  return ;
	}

	if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK)
	{
		Error_Handler();
	}


	if (HAL_RCC_ClockConfig(&Clock_Init, flash_latency) != HAL_OK)
	{
		Error_Handler();
	}

	/*Configure the systick timer interrupt frequency (for every 1 ms) */
	uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
	HAL_SYSTICK_Config(hclk_freq/1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}
void Error_Handler()
{
	while(1);
}

void UART2_Init()
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if ( HAL_UART_Init(&huart2) != HAL_OK )
	{
		//There is a problem
		Error_Handler();
	}

}


void GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	//for input user button in first board
	GPIO_InitTypeDef GPIO_BTN;
	GPIO_BTN.Pin = GPIO_PIN_0;
	GPIO_BTN.Mode = GPIO_MODE_IT_RISING;
	GPIO_BTN.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &GPIO_BTN);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	GPIO_InitTypeDef gpio_led;

	gpio_led.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_led.Pin = GPIO_PIN_12;
	gpio_led.Pull = GPIO_NOPULL;
	gpio_led.Speed = GPIO_SPEED_MEDIUM;

	HAL_GPIO_Init(GPIOD, &gpio_led);

}

void RTC_Init()
{
	hRTC.Instance = RTC;
	hRTC.Init.HourFormat = RTC_HOURFORMAT_24;
	hRTC.Init.AsynchPrediv = 0x7F;
	hRTC.Init.SynchPrediv = 0xFF;
	hRTC.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hRTC.Init.OutPutType = RTC_OUTPUT_TYPE_PUSHPULL;
	hRTC.Init.OutPut = RTC_OUTPUT_DISABLE;

	if(HAL_RTC_Init(&hRTC)!= HAL_OK)
	{
		Error_Handler();
	}
}

void RTC_Calendar_Config()
{
	//Let's configure the time as 12 : 11 : 10 PM and the date as 20th April 2024 Saturday
	RTC_TimeTypeDef time_config;

	time_config.Hours = 23;
	time_config.Minutes = 15;
	time_config.Seconds = 20;
	//time_config.TimeFormat = RTC_HOURFORMAT12_AM;

	if(HAL_RTC_SetTime(&hRTC, &time_config, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	RTC_DateTypeDef date_config;

	date_config.Date = 20;
	date_config.Month = RTC_MONTH_APRIL;
	date_config.Year = 24;
	date_config.WeekDay = RTC_WEEKDAY_SUNDAY;

	if(HAL_RTC_SetDate(&hRTC, &date_config, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	RTC_Calendar_Config();

	RTC_TimeTypeDef get_time;

	HAL_RTC_GetTime(&hRTC, &get_time, RTC_FORMAT_BIN);


	printmsg("Current Time is %02d:%02d:%02d\r\n", get_time.Hours, get_time.Minutes, get_time.Seconds );

	RTC_DateTypeDef get_date;

	HAL_RTC_GetDate(&hRTC, &get_date, RTC_FORMAT_BIN);

	printmsg("Current Date is: %02d-%02d-%02d<%s>  <%s> \r\n" , get_date.Month, get_date.Date, get_date.Year, getAM_PM(get_date.WeekDay), getDayofWeek(get_date.WeekDay));

	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_RTC_ALARM_CLEAR_FLAG(&hRTC, RTC_FLAG_ALRAF);

	RTC_Alarm_Config();

	printmsg("went to standby mode\r\n");

	HAL_PWR_EnterSTANDBYMode();
}

void RTC_Alarm_Config()
{
	RTC_AlarmTypeDef alarm_config;
	memset(&alarm_config, 0, sizeof(alarm_config));

	HAL_RTC_DeactivateAlarm(&hRTC, RTC_ALARM_A);

	alarm_config.Alarm = RTC_ALARM_A;
	alarm_config.AlarmTime.Hours = 23;
	alarm_config.AlarmTime.Minutes = 15;
	alarm_config.AlarmTime.Seconds = 30;
//	alarm_config.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
	//alarm_config.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
	//alarm_config.AlarmDateWeekDay = RTC_WEEKDAY_SUNDAY;
	alarm_config.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	alarm_config.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;

	if(HAL_RTC_SetAlarm_IT(&hRTC, &alarm_config, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	printmsg("Alarm Set successful\r\n");
}


char* getDayofWeek(uint8_t number)
{
	char* WeekDay [ ]= {"Monday",  "Tuesday" , "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

	return WeekDay[number-1];
}

char* getAM_PM(uint8_t number)
{
	char* AM_PM [] = { "AM", "PM" };

	return AM_PM[number];
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	printmsg("Alarm Activated");

	RTC_Calendar_Config();

	RTC_TimeTypeDef get_time;

	HAL_RTC_GetTime(&hRTC, &get_time, RTC_FORMAT_BIN);

	//DUMMY CALLS
	RTC_DateTypeDef get_date;

	HAL_RTC_GetDate(&hRTC, &get_date, RTC_FORMAT_BIN);


	printmsg("Current Time is %02d:%02d:%02d\r\n", get_time.Hours, get_time.Minutes, get_time.Seconds );
	//turn on LED
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

	HAL_Delay(2000); //malfunctions if current isr's priority is higher than systick. Because delay function depends on systick

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

}
