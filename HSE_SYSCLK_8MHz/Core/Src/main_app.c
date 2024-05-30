#include "main_app.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define FALSE	0
#define TRUE	1

void SystemClockConfig(void);
void UART2_Init();
void Error_Handler();


UART_HandleTypeDef huart2;
uint8_t rcvd_it;
uint8_t data_buffer [100];
uint8_t RxComplt = FALSE;
uint16_t count = 0;


char *user_data = " The application is running\r\n";

int main(void)
{
	HAL_Init();

	SystemClockConfig();

	UART2_Init();

	uint16_t len_data = strlen(user_data);

	HAL_UART_Transmit(&huart2, (uint8_t*) user_data,len_data , HAL_MAX_DELAY);

	while (RxComplt != TRUE)
	{
	HAL_UART_Receive_IT(&huart2, &rcvd_it, 1);
	}

	return 0;
}

void SystemClockConfig(void)
{

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

	if( HAL_UART_Init(&huart2) != HAL_OK)
	{
		//error
		Error_Handler();
	}

}


void Error_Handler()
{
	while(1);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(rcvd_it == '\r')
	{
		RxComplt = TRUE;
		data_buffer[count++] = '\r';
		HAL_UART_Transmit(&huart2, (uint8_t*) data_buffer,count , HAL_MAX_DELAY);
	}
	else
		data_buffer[count++] = rcvd_it;
}
