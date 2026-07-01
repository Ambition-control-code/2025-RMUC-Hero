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
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "spi.h"
#include "tim.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//使用KEIL V5编译  V6ccrmam编译不通过
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
uint8_t canrx[8];

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//总控制在u_i_pid.c
/*串极PID响应差*/
	//假设输入24v，输出30v则最大占空比1+（30-24）/30=1.2
	//最小占空比不小于mos开关死区即可
//	//void pid_init(PID *pid,    float kp,float ki,float kd, float omax,float omin, float pmax, float imax, float dmax)
//	//     电流环PID参数初始化     P          I          D       PWM调节上限  PWM调节下限    
//	       pid_init(&CAP_charge_I,0.0000025,0.00    ,0.00000    ,1.20       ,0.05      , 0.01       ,0     ,0.001); 
//  //     电压环PID参数初始化     P          I          D       电流调节上限 电流调节下限 ma
//	       pid_init(&CAP_charge_V,0.0005    ,0.0001    ,0.0       ,8000       ,-8000      , 80       ,80       ,80);
//  //     功率环PID参数初始化     P          I          D       电压调节上限 电压调节下限 mv				 
//	       pid_init(&CAP_charge_P,0.05    ,0.00     ,0.0000     ,15000      ,5000       , 150       ,0         ,50);
 
/*并联PID*/
	//void pid_init(PID *pid,    float kp,float ki,float kd, float omax,float omin, float pmax, float imax, float dmax)
	//     电流环PID参数初始化     P          I          D       PWM调节上限  PWM调节下限    
	       pid_init(&CAP_charge_I,0.000006  ,0.00    ,0.00000    ,1.20       ,0.10f       , 0.7       ,0.1       , 0.01); 
  //     电压环PID参数初始化     P          I          D       PWM调节上限 电流调节下限 ma
	       pid_init(&CAP_charge_V,0.000006  ,0.01    ,0.00000      ,1.20f       ,0.10f      , 0.005    ,0       , 0.01);
  //     功率环PID参数初始化     P          I          D       充电调节上限 放电功率上限 W				 
	       pid_init(&CAP_charge_P,0.9    ,0.00  ,0.0000     ,15000.0f    ,-15000.0f       , 800       ,0          ,200);
				 
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
  /* USER CODE BEGIN 2 */
	HAL_Delay(100);
	
  LCD_INIT();
	
	HAL_Delay(100);
	ADC_INIT();
	
	U_I_CTRL_INIT();
	
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
	

	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(1);
		LCD_ShowString(0,0,"CL_I:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(32,0,"-",(POW_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowIntNum(40,0,(POW_I_ADC_DATA.lowflt_out>0?POW_I_ADC_DATA.lowflt_out:-POW_I_ADC_DATA.lowflt_out),5,WHITE,BLACK,12);
		
		LCD_ShowString(0,20,"DC_U:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowIntNum(40,20,POW_U_ADC_DATA.lowflt_out,5,WHITE,BLACK,12);
		
		LCD_ShowString(80,0,"CP_I:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(80+32,0,"-",(CAP_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowIntNum(120,0,CAP_I_ADC_DATA.lowflt_out>0?CAP_I_ADC_DATA.lowflt_out:-CAP_I_ADC_DATA.lowflt_out,5,WHITE,BLACK,12);
			
		LCD_ShowString(80,20,"CP_U:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowIntNum(120,20,CAP_U_ADC_DATA.lowflt_out,5,WHITE,BLACK,12);

		LCD_ShowString(0,40,"CP_P:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(0+32,40,"-",(CAP_P>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowFloatNum1(48,40,abs_user(CAP_P),5,WHITE,BLACK,12);
	
    LCD_ShowString(0,60,"CL_P:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(0+32,60,"-",(POW_P>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowFloatNum1(48,60,abs_user(POW_P),5,WHITE,BLACK,12);
		
	  if(protection_mode)
		{
		   LCD_ShowString(0,55,"POWER_PR0TECTION",BRRED,BLACK,12,0);
			if(protection_mode==1||protection_mode==3)  //过压保护
			{
			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(500);
			  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(500);
			}
			if(protection_mode==5)
			{
			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(1000);
			  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(1000);
			}
			if(protection_mode==4||protection_mode==2) //过流保护
			{
			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(200);
			  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(200);
			}
		}
		
		//fdcan_transmit(&hfdcan2, &TxHeader2, 0x301, FDCAN_DLC_BYTES_8, canrx);
		//fdcan_transmit(&hfdcan3, &TxHeader3, 0x303, FDCAN_DLC_BYTES_8, canrx);
	
		//底盘输出电流为-
		//电容充电电流为+
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
