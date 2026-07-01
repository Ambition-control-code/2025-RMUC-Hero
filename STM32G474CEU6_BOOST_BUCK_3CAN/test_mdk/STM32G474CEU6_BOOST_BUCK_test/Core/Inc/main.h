/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "lcd_init.h"

#include "power_measurement.h"
#include "power_protection.h"

#include "u_i_pid.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern SPI_HandleTypeDef hspi1;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
	
extern HRTIM_HandleTypeDef hhrtim1;

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;

extern FDCAN_TxHeaderTypeDef TxHeader1;
extern FDCAN_RxHeaderTypeDef RxHeader1;
extern FDCAN_TxHeaderTypeDef TxHeader2;
extern FDCAN_RxHeaderTypeDef RxHeader2;
extern FDCAN_TxHeaderTypeDef TxHeader3;
extern FDCAN_RxHeaderTypeDef RxHeader3;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define CCMRAM __attribute__((section("ccmram")))
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void fdcan_transmit(FDCAN_HandleTypeDef *hfdcan,FDCAN_TxHeaderTypeDef *TxHeader,uint32_t can_id, uint32_t DataLength, uint8_t tx_data[]);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOA
#define DC_Pin GPIO_PIN_6
#define DC_GPIO_Port GPIOA
#define RES_Pin GPIO_PIN_4
#define RES_GPIO_Port GPIOC
#define BLK_Pin GPIO_PIN_0
#define BLK_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_2
#define CS_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
