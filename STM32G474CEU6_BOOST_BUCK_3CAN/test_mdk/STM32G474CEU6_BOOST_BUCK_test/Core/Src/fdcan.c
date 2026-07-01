/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_RxHeaderTypeDef RxHeader1;
FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_RxHeaderTypeDef RxHeader2;
FDCAN_TxHeaderTypeDef TxHeader3;
FDCAN_RxHeaderTypeDef RxHeader3;
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = ENABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 17;
  hfdcan2.Init.NominalSyncJumpWidth = 4;
  hfdcan2.Init.NominalTimeSeg1 = 5;
  hfdcan2.Init.NominalTimeSeg2 = 4;
  hfdcan2.Init.DataPrescaler = 17;
  hfdcan2.Init.DataSyncJumpWidth = 4;
  hfdcan2.Init.DataTimeSeg1 = 5;
  hfdcan2.Init.DataTimeSeg2 = 4;
  hfdcan2.Init.StdFiltersNbr = 28;
  hfdcan2.Init.ExtFiltersNbr = 8;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
   FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x321;
  sFilterConfig.FilterID2 = 0x123;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0) != HAL_OK) { Error_Handler(); }
  /* Prepare Tx Header */
//  TxHeader1.Identifier = 0x321;
//  TxHeader1.IdType = FDCAN_STANDARD_ID;
//  TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
//  TxHeader1.DataLength = FDCAN_DLC_BYTES_8;
//  TxHeader1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//  TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;
//  TxHeader1.FDFormat = FDCAN_CLASSIC_CAN;
//  TxHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//  TxHeader1.MessageMarker = 0;
  /* USER CODE END FDCAN2_Init 2 */

}
/* FDCAN3 init function */
void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = ENABLE;
  hfdcan3.Init.TransmitPause = ENABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 17;
  hfdcan3.Init.NominalSyncJumpWidth = 4;
  hfdcan3.Init.NominalTimeSeg1 = 4;
  hfdcan3.Init.NominalTimeSeg2 = 5;
  hfdcan3.Init.DataPrescaler = 17;
  hfdcan3.Init.DataSyncJumpWidth = 4;
  hfdcan3.Init.DataTimeSeg1 = 5;
  hfdcan3.Init.DataTimeSeg2 = 4;
  hfdcan3.Init.StdFiltersNbr = 28;
  hfdcan3.Init.ExtFiltersNbr = 8;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */
 if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0) != HAL_OK) { Error_Handler(); }
  /* USER CODE END FDCAN3_Init 2 */

}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN3)
  {
  /* USER CODE BEGIN FDCAN3_MspInit 0 */

  /* USER CODE END FDCAN3_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN3 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN3 GPIO Configuration
    PB3     ------> FDCAN3_RX
    PB4     ------> FDCAN3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_FDCAN3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN FDCAN3_MspInit 1 */

  /* USER CODE END FDCAN3_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN3)
  {
  /* USER CODE BEGIN FDCAN3_MspDeInit 0 */

  /* USER CODE END FDCAN3_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN3 GPIO Configuration
    PB3     ------> FDCAN3_RX
    PB4     ------> FDCAN3_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4);

  /* USER CODE BEGIN FDCAN3_MspDeInit 1 */

  /* USER CODE END FDCAN3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void fdcan_transmit(FDCAN_HandleTypeDef *hfdcan,FDCAN_TxHeaderTypeDef *TxHeader,uint32_t can_id, uint32_t DataLength, uint8_t tx_data[])
{	
	
	TxHeader->Identifier = can_id;
	TxHeader->IdType = FDCAN_EXTENDED_ID;
	if(can_id < 0x800) {	//exactly not right
		TxHeader->IdType = FDCAN_STANDARD_ID;
	}
	TxHeader->TxFrameType = FDCAN_DATA_FRAME;
	TxHeader->DataLength = DataLength;
	TxHeader->ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TxHeader->BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader->FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader->MessageMarker = 0;	//marker++;
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, TxHeader, tx_data) != HAL_OK)//或者换while失败后初始化
        {
//		fdcan1_send_fail.flag = 1;
//		memcpy(&fdcan1_send_fail.TxHeader, &TxHeader1, sizeof(FDCAN_TxHeaderTypeDef));
//		memcpy(fdcan1_send_fail.TxData, tx_data, can_dlc2len(DataLength));
	}
}
/* USER CODE END 1 */
