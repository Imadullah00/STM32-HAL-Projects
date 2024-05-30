/*
 * CAN_main.c
 *
 *  Created on: Apr 14, 2024
 *      Author: ImadF
 */
#include "CAN_main.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include <stdio.h>

void Error_Handler(void);
void UART2_Init();
void SystemClock_Config_HSE(uint8_t clock_freq);
void GPIO_Init();
void CAN1_Init();
void CAN1_Tx();
void CAN1_Rx(void);
void CAN_Filter_Config(void);

CAN_HandleTypeDef hCAN1;
UART_HandleTypeDef huart2;

int main()
{

	HAL_Init();

	SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);

	GPIO_Init();

	UART2_Init();

	CAN1_Init();

	CAN_Filter_Config();

	if(HAL_CAN_ActivateNotification(&hCAN1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_CAN_Start(&hCAN1) != HAL_OK)
	{
		Error_Handler();
	}

	CAN1_Tx();

	//CAN1_Rx();
	while(1);


	return 0;
}
void SystemClock_Config_HSE(uint8_t clock_freq)

{

	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;
    uint8_t flash_latency=0;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	Osc_Init.HSEState = RCC_HSE_ON;
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

void Error_Handler(void)
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
	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_5;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);
}

void CAN1_Init()
{
	hCAN1.Instance = CAN1;
	hCAN1.Init.AutoBusOff  = ENABLE;
	hCAN1.Init.AutoRetransmission = ENABLE;
	hCAN1.Init.AutoWakeUp = DISABLE;
	hCAN1.Init.Mode = CAN_MODE_LOOPBACK;
	hCAN1.Init.ReceiveFifoLocked = DISABLE;
	hCAN1.Init.TransmitFifoPriority = DISABLE;
	hCAN1.Init.TimeTriggeredMode = DISABLE;

	//configure bit timing
	hCAN1.Init.Prescaler = 5;
	hCAN1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hCAN1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hCAN1.Init.TimeSeg2 = CAN_BS1_1TQ;

	if(HAL_CAN_Init(&hCAN1) != HAL_OK)
	{
		Error_Handler();
	}

}

void CAN1_Tx()
{
	uint8_t our_message [5] = { 'H' , 'E' , 'L' , 'L' , 'O' };
	uint32_t mailbox;


	CAN_TxHeaderTypeDef TxHeader;

	TxHeader.StdId = 0x65D;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 5;
	TxHeader.RTR = CAN_RTR_DATA;

	if(HAL_CAN_AddTxMessage(&hCAN1, &TxHeader, our_message, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

}

void CAN1_Rx(void)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rcvd_msg[5];
	char msg[50];

	//wait for atleast one message in the FIFO0
	while(! HAL_CAN_GetRxFifoFillLevel(&hCAN1, CAN_RX_FIFO0));

	if(HAL_CAN_GetRxMessage(&hCAN1, CAN_RX_FIFO0, &RxHeader, rcvd_msg) != HAL_OK)
	{
		Error_Handler();
	}

	sprintf(msg, "Message Received: %s\r\n" , rcvd_msg);

	HAL_UART_Transmit(&huart2, (uint8_t*)&msg, strlen(msg), HAL_MAX_DELAY);

}


void CAN_Filter_Config(void)
{
	CAN_FilterTypeDef can_filterh;

	can_filterh.FilterActivation = CAN_FILTER_ENABLE;
	can_filterh.FilterBank = 0;
	can_filterh.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filterh.FilterIdHigh = 0x0000;
	can_filterh.FilterIdLow = 0x0000;
	can_filterh.FilterMaskIdHigh = 0x0000;
	can_filterh.FilterMaskIdLow = 0x0000;
	can_filterh.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filterh.FilterScale = CAN_FILTERSCALE_32BIT;

	if(HAL_CAN_ConfigFilter(&hCAN1, &can_filterh) != HAL_OK)
	{
		Error_Handler();
	}
}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	char umessage [50];
	sprintf(umessage, "Transmission Successful: M0 mailbox\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) umessage, strlen(umessage), HAL_MAX_DELAY);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	char umessage [50];
	sprintf(umessage, "Transmission Successful: M1\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) umessage, strlen(umessage), HAL_MAX_DELAY);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	char umessage [50];
	sprintf(umessage, "Transmission Successful: M2\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) umessage, strlen(umessage), HAL_MAX_DELAY);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rcvd_msg[5];
	char msg[50];

	if(HAL_CAN_GetRxMessage(&hCAN1, CAN_RX_FIFO0, &RxHeader, rcvd_msg) != HAL_OK)
	{
		Error_Handler();
	}

	sprintf(msg, "Message Received: %s\r\n" , rcvd_msg);

	HAL_UART_Transmit(&huart2, (uint8_t*)&msg, strlen(msg), HAL_MAX_DELAY);

}


void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	char msg[50];
	sprintf(msg, "CAN Error Detected");
	HAL_UART_Transmit(&huart2, (uint8_t*)&msg, strlen(msg), HAL_MAX_DELAY);

}
