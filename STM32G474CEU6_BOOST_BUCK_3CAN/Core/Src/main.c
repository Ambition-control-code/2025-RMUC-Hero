/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//ʹ��KEIL V5����  V6ccrmam���벻ͨ��
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t canrx[8]={1,2,3,4,5,6,7,8};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	//FLASH����
	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();//启用Flash预取缓冲区
	//��������cache��ָ��cache
	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();//启用指令缓存
  __HAL_FLASH_DATA_CACHE_ENABLE();//启用数据缓存
	//�رճ���ΪO����  0/0=0;
	SCB->CCR=SCB->CCR&0xFFFFFFEF;//关闭除0错误
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//�ܿ�����u_i_pid.c
/*����PID��Ӧ��*/
	//��������24v�����30v�����ռ�ձ�1+��30-24��/30=1.2
	//��Сռ�ձȲ�С��mos������������
//	//void pid_init(PID *pid,    float kp,float ki,float kd, float omax,float omin, float pmax, float imax, float dmax)
//	//     ������PID������ʼ��     P          I          D       PWM��������  PWM��������    
//	       pid_init(&CAP_charge_I,0.0000025,0.00    ,0.00000    ,1.20       ,0.05      , 0.01       ,0     ,0.001); 
//  //     ��ѹ��PID������ʼ��     P          I          D       ������������ ������������ ma
//	       pid_init(&CAP_charge_V,0.0005    ,0.0001    ,0.0       ,8000       ,-8000      , 80       ,80       ,80);
//  //     ���ʻ�PID������ʼ��     P          I          D       ��ѹ�������� ��ѹ�������� mv				 
//	       pid_init(&CAP_charge_P,0.05    ,0.00     ,0.0000     ,15000      ,5000       , 150       ,0         ,50);
 
/*����PID*/
//	//void pid_init(PID *pid,    float kp,float ki,float kd, float omax,float omin, float pmax, float imax, float dmax)
//	//     ������PID������ʼ��     P          I          D       PWM��������  PWM��������    
//	       pid_init(&CAP_charge_I,0.0000039 ,0.00000   ,0.0000019    ,1.20       ,0.10       , 0.09      ,0.1   , 0.08); 
//  //     ��ѹ��PID������ʼ��     P          I          D       PWM�������� ������������ 
//	       pid_init(&CAP_charge_V,0.000003  ,0.00      ,0.0001 ,1.20       ,0.10       , 0.05  ,0          , 0.001);
//  //     ���ʻ�PID������ʼ��     P          I          D       ������������ ������������ ma				 
//	       pid_init(&CAP_charge_P,0.35      ,0.0001    ,0.001 ,15000      ,-25000    , 1000    ,100          ,500);
//	//     ����������PID������ʼ��     P          I          D   ���ʵ������� ���ʵ������� ma				 
//	       pid_init(&CAP_buffer_P,0.3       ,0.0000     ,0.07 ,10          ,-5      ,7         ,5           ,5);	
				 
				 	//void pid_init(PID *pid,    float kp,float ki,float kd, float omax,float omin, float pmax, float imax, float dmax)
	//     ������PID������ʼ��     P          I          D       PWM��������  PWM��������    
	       pid_init(&CAP_charge_I,0.000005 ,0.0000039   ,0.0000019   ,1.20       ,0.10       , 0.09      ,0.09   , 0.08); 
  //     ��ѹ��PID������ʼ��     P          I          D       PWM�������� ������������ 
	       pid_init(&CAP_charge_V,0.000005  ,0.000003    ,0.0001 ,1.20       ,0.10       , 0.05      ,0.05   , 0.001);
  //     ���ʻ�PID������ʼ��     P          I          D       ������������ ������������ ma				 
	       pid_init(&CAP_charge_P,15      ,0.5    ,0.001 ,15000      ,-20000    , 5000    ,5000          ,500);
	//     ����������PID������ʼ��     P          I          D   ���ʵ������� ���ʵ������� ma				 
	       pid_init(&CAP_buffer_P,0.3       ,0.0000     ,0.07 ,10          ,-5      ,7         ,5           ,5);	
		//电容充电电流环，充电电压环，充电功率环初始化
		//缓冲能量闭环



	Motor_PID_init();//底盘电机pid初始化//没用的东西
	INIT_FLASH_DATA();//有关Flash的初始化内容，并从Flash中读取数据判断是否控制底盘
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_HRTIM1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_ADC3_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
	pictureInit();//UI初始化
	
	HAL_Delay(100);
	
	LCD_INIT();//显示屏初始化
	
	HAL_Delay(100);
	ADC_INIT();//ADC初始化
	
	USART_SET_INIT();//串口初始化
	CAN_BSP_INIT();//CAN初始化
	
	U_I_CTRL_INIT();

//LED开启
  LED1_ON;
	LED2_ON;
	
	UI_init();//显示屏有关内容

	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(20);
	  UI_RUN();//显示屏有关内容		
		//����can_usartͨ��
//		fdcan_transmit(&hfdcan1, &TxHeader1, 0x301, FDCAN_DLC_BYTES_8, canrx);
//		fdcan_transmit(&hfdcan2, &TxHeader2, 0x302, FDCAN_DLC_BYTES_8, canrx);
//		fdcan_transmit(&hfdcan3, &TxHeader3, 0x303, FDCAN_DLC_BYTES_8, canrx);	
//	  HAL_UART_Transmit_DMA(&huart1,canrx,8);
//		HAL_UART_Transmit_DMA(&huart3,canrx,8);
		//�����������Ϊ-
		//���ݳ�����Ϊ+
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	LED1_OFF;
	LED2_OFF;
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
