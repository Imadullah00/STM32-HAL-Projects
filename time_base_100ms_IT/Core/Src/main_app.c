#include "main_app.h"
#include "stm32f4xx_hal.h"
#include <string.h>

void SystemClockConfig(void);
void Error_Handler();
void TIMER_Init(void);
void GPIO_Init(void);


TIM_HandleTypeDef htimer6;

int main(void)
{
	HAL_Init();

	SystemClockConfig();

	GPIO_Init();

	TIMER_Init();

	HAL_TIM_Base_Start_IT(&htimer6);

	while(1);



	return 0;
}

void SystemClockConfig(void)
{

}



void Error_Handler()
{
	while(1);
}

void GPIO_Init(void)
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_LED;
	GPIO_LED.Pin = GPIO_PIN_12;
	GPIO_LED.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_LED.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_LED);
}


void TIMER_Init(void)
{
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 60;
	htimer6.Init.Period = 64000-1;

	if(HAL_TIM_Base_Init(&htimer6) != HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}
