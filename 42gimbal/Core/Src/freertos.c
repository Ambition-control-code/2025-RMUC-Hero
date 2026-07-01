/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "gimbal_task.h"
#include "vision_task.h"
#include "balance_task.h"
#include "refree_task.h"
#include "imu_task.h"
#include "vision_task.h"
#include "ulitility_tools.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart1,huart2;
static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
LowPassFilter2p_Info_TypeDef INS_AcceLPF2p[3];  

float INS_Angle[3] = {0.0f, 0.0f, 0.0f}; 
float MSG[3] = {0.0f, 0.0f, 0.0f}; 
uint8_t recieve_can[9];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern SPI_HandleTypeDef hspi2;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
CAN_RxHeaderTypeDef	RxHeader;     //½ÓÊÕ
bucket_imu imu;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void QuaternionToEularAngle(float* q, float* Yaw, float* Pitch, float* Roll) {
	
	
*Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) *
57.295779513f;
*Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) *
57.295779513f;
*Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void refree_task(void const * argument);
extern void gimbal_task(void const * argument);
extern void vision_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, refree_task, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, gimbal_task, osPriorityIdle, 0, 512);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, vision_task, osPriorityIdle, 0, 512);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	imu.flash_flag=1;
	imu.q[0]=1;
    //set spi frequency
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
		 BMI088_read(imu.gyro,imu.acc,&(imu.temp));
		
		 LowPassFilter2p_Init(&INS_AcceLPF2p[0],INS_LPF2p_Alpha);
     LowPassFilter2p_Init(&INS_AcceLPF2p[1],INS_LPF2p_Alpha);
     LowPassFilter2p_Init(&INS_AcceLPF2p[2],INS_LPF2p_Alpha);
		
//		 AHRS_init(imu.q,imu.acc,MSG);
						
//	uint32_t HAIHAIHAI =osKernelSysTick();		
  for(;;)
  {
	 BMI088_read(imu.gyro,imu.acc,&(imu.temp));
		
    imu.acc_fliter[0] = LowPassFilter2p_Update(&INS_AcceLPF2p[0],imu.acc[0]);
    imu.acc_fliter[1] = LowPassFilter2p_Update(&INS_AcceLPF2p[2],imu.acc[1]);
    imu.acc_fliter[2] = LowPassFilter2p_Update(&INS_AcceLPF2p[1],imu.acc[2]);	
		
		int ii;
		if(imu.cnt==20000){imu.cnt=20000;}
		else {imu.cnt++;}
		if(!imu.flash_flag)
		{		
			if(imu.cnt>=1000 && imu.cnt<20000)				
			{			
				int i;
				for(i = 0; i<=2; i++)
				imu.gyro_zero[i]+=imu.gyro[i];		
			}			
			for(ii = 0; ii<=2; ii++)
			imu.gyro_buff[ii] = imu.gyro[ii] - imu.gyro_zero[ii]/18999;			
			if(imu.cnt==20000&&imu.flash_read_flag==0)
			{
				STMFLASH_Write(0X08040000,(u32*)imu.gyro_zero,3); 
				imu.flash_read_flag++;
			}	     
		}
		else if(imu.flash_flag)
		{			
			imu.cnt=20000;			
			if (!imu.flash_read_flag)
			{	
				imu.flash_read_flag=1;				
				STMFLASH_Read(0X08020000,(u32*)imu.gyro_zero,3);
				if(imu.gyro_zero[1]==0)
				{
					imu.gyro_zero[0] = 55.2097893;
					imu.gyro_zero[1] = -15.706131;
					imu.gyro_zero[2] = -17.9081974;
				}
				for(ii = 0; ii<=2; ii++)
				imu.gyro_buff[ii] = imu.gyro[ii] - imu.gyro_zero[ii]/18999;
			}
			else 
			{
				for(ii = 0; ii<=2; ii++)
				imu.gyro_buff[ii] = imu.gyro[ii] - imu.gyro_zero[ii]/18999;
			}		
		}			
		if(imu.cnt==20000)
		{
		    imu_task();		 
		    imu.yaw[1]=imu.yaw[0];		 		  
			QuaternionToEularAngle(imu.q,&imu.yaw[0],&imu.pitch,&imu.roll);
		    float diff=imu.yaw[0]-imu.yaw[1];
			if(diff<= -150)
			imu.yaw_cnt++;
			else if(diff>= 150)
			imu.yaw_cnt--;
			imu.yaw[2]=imu.yaw[0]+imu.yaw_cnt*360.0f;
		}
//	 osDelayUntil(&HAIHAIHAI,1);
		osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
