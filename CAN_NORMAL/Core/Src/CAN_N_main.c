/*
 * CAN_N_MAIN.C
 *
 *  Created on: Apr 16, 2024
 *      Author: ImadF
 */

#include "CAN_N_main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

void SystemClock_Config_HSE(uint8_t clock_freq);
void Error_Handler();
void TIMER_Init(void);
void GPIO_Init(void);
void CAN1_Init();
void CAN1_Tx();
void CAN_Filter_Config(void);
void UART2_Init();
void Send_response(uint8_t StdID);
void LED_Manage_Output(uint8_t led_no);

CAN_TxHeaderTypeDef TxHeader; // for SendResponse()

TIM_HandleTypeDef htimer6;
CAN_HandleTypeDef hCAN1;
UART_HandleTypeDef huart2;

uint8_t tim_cnt = 0;

int main(void)
{
	HAL_Init();

	SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);

	GPIO_Init();

	UART2_Init();

	TIMER_Init();

	CAN1_Init();

	CAN_Filter_Config();

	if(HAL_CAN_ActivateNotification(&hCAN1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF) != HAL_OK)
	{
		Error_Handler();
	}


	/*while(1)
	{
		while(! (TIM6->SR & TIM_SR_UIF) );
		TIM6->SR = 0; // clear the UIF field
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	}*/

	HAL_CAN_Start(&hCAN1);

	return 0;
}

void Error_Handler()
{
	while(1);
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

	//for output LEDs (on board) in second board
	GPIO_InitTypeDef GPIO_LEDs;
	GPIO_LEDs.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_LEDs.Pull = GPIO_NOPULL;

	GPIO_LEDs.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOD, &GPIO_LEDs);

	GPIO_LEDs.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOD, &GPIO_LEDs);

	GPIO_LEDs.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOD, &GPIO_LEDs);

	GPIO_LEDs.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOD, &GPIO_LEDs);

	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}


void TIMER_Init(void)
{
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler =  4999 ;
	htimer6.Init.Period = 10000-1;

	if(HAL_TIM_Base_Init(&htimer6) != HAL_OK)
	{
		Error_Handler();
	}

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

void SystemClock_Config_HSE(uint8_t clock_freq)
{

	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;
	uint8_t flash_latency = 0;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_HSI ;
	Osc_Init.HSEState = RCC_HSE_ON;
	Osc_Init.LSEState = RCC_LSE_ON;
	Osc_Init.HSIState = RCC_HSI_ON;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(tim_cnt == 4)
	{
		CAN_TxHeaderTypeDef TxHeader2;
		uint32_t mailbox2;
		uint8_t message2 =0; // will be discarded  by CAN  controller as its an RTR frame

		// above 3 lines of code were outside the if block in instructor's code
		// don't think it makes a difference

		TxHeader2.DLC = 2;
		TxHeader2.IDE = CAN_ID_STD;
		TxHeader2.RTR = CAN_RTR_DATA;
		TxHeader2.StdId = 0x651;

		if(HAL_CAN_AddTxMessage(&hCAN1, &TxHeader2, &message2, &mailbox2) != HAL_OK)
		{
			Error_Handler();
		}

		tim_cnt = 0;

	}
	else
	{
		CAN1_Tx();
		tim_cnt++;
	}
}
void CAN1_Init()
{
	hCAN1.Instance = CAN1;
	hCAN1.Init.AutoBusOff  = DISABLE;
	hCAN1.Init.AutoRetransmission = ENABLE;
	hCAN1.Init.AutoWakeUp = DISABLE;
	hCAN1.Init.Mode = CAN_MODE_NORMAL;
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
uint8_t led_no = 0;

void CAN1_Tx()
{
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.DLC =1;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x65D;

	uint32_t mailbox;
	uint8_t message;

	message = ++led_no;

	if(led_no == 4)
	{
		led_no = 0;
	}

	if(HAL_CAN_AddTxMessage(&hCAN1, &TxHeader, &message, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

}
void CAN_Filter_Config(void)
{
	CAN_FilterTypeDef can_filterh;

	can_filterh.FilterActivation = CAN_FILTER_ENABLE;
	can_filterh.FilterBank = 0;
	can_filterh.FilterFIFOAssignment = CAN_FILTER_FIFO0;
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
	sprintf(umessage, "Transmission Successful: M0\r\n");
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

	uint8_t rcvd_msg[8];
	char msg[50];

	if(HAL_CAN_GetRxMessage(&hCAN1, CAN_RX_FIFO0, &RxHeader, rcvd_msg)!= HAL_OK)
	{
		Error_Handler();
	}

	if(RxHeader.StdId == 0x65D && RxHeader.RTR == 0)
	{
		//n2 receives data containing LED number from n1
		LED_Manage_Output(led_no);
		sprintf(msg, "Message Received: #%X\r\n", rcvd_msg[0]);

	}

	if(RxHeader.StdId == 0x65D && RxHeader.RTR == 1)
	{
		//n2 receives remote from n1 asking for 2 bytes of data
		Send_response(RxHeader.StdId);
		return;
	}

	if(RxHeader.StdId == 0x651 && RxHeader.RTR == 0)
	{
		//n1 receives from n2. This should be the data it requested
		sprintf(msg, "Reply Received: #%X\r\n", rcvd_msg[0] << 8 | rcvd_msg[1]);
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)&msg, strlen(msg), HAL_MAX_DELAY);

}


void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	char msg[50];
	sprintf(msg, "CAN Error Detected");
	HAL_UART_Transmit(&huart2, (uint8_t*)&msg, strlen(msg), HAL_MAX_DELAY);
}

void Send_response(uint8_t StdID)
{

	uint32_t MailBox;

	uint8_t response [2] = { 0xAB, 0xCD };

	TxHeader.DLC = 2;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = StdID;
	TxHeader.IDE = CAN_ID_STD;

	if(HAL_CAN_AddTxMessage(&hCAN1, &TxHeader, response, &MailBox) != HAL_OK)
	{
		Error_Handler();
	}


}
void LED_Manage_Output(uint8_t led_no)
{
	switch(led_no)
	{
	case 1:
	{
		//switch on LED_GREEN (rest all OFF)
		HAL_GPIO_WritePin(GPIOD, LED_GREEN, SET);
		HAL_GPIO_WritePin(GPIOD, LED_ORANGE, RESET);
		HAL_GPIO_WritePin(GPIOD, LED_RED, RESET);
		HAL_GPIO_WritePin(GPIOD, LED_BLUE, RESET);
		break;

	}

	case 2:
	{
		//switch on LED_ORANGE
		HAL_GPIO_WritePin(GPIOD, LED_GREEN, RESET);
		HAL_GPIO_WritePin(GPIOD, LED_ORANGE, SET);
		HAL_GPIO_WritePin(GPIOD, LED_RED, RESET);
		HAL_GPIO_WritePin(GPIOD, LED_BLUE, RESET);
		break;
	}

	case 3:
	{
		//switch on LED_RED
		HAL_GPIO_WritePin(GPIOD, LED_GREEN, RESET);
		HAL_GPIO_WritePin(GPIOD, LED_ORANGE, RESET);
		HAL_GPIO_WritePin(GPIOD, LED_RED, SET);
		HAL_GPIO_WritePin(GPIOD, LED_BLUE, RESET);
		break;
	}

	case 4:
	{
		//switch on LED_BLUE
		HAL_GPIO_WritePin(GPIOD, LED_GREEN, RESET);
		HAL_GPIO_WritePin(GPIOD, LED_ORANGE, RESET);
		HAL_GPIO_WritePin(GPIOD, LED_RED, RESET);
		HAL_GPIO_WritePin(GPIOD, LED_BLUE, SET);
		break;
	}
	default:
		Error_Handler();
	}

}

