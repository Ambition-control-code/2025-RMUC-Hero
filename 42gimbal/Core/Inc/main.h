/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define u32 uint32_t
#define vu32 volatile uint32_t
#define u8 char   
#define wait_for_Power 2000

#include "stm32f4xx_hal_tim.h"
#include "AHRS.h"
#include "BMI088driver.h"
#include "math.h"
#include "stmflash.h"
#include "ADRC_core.h"
//#include "tim.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef __packed struct
{
    float Input;

    float h0;
    float r;

    float x;
    float dx;
    float ddx;

    float last_dx;
    float last_ddx;

    uint32_t DWT_CNT;
    float dt;
} TD_t;

typedef struct{
u8 flash_flag,flash_read_flag;
float acc[3],acc_fliter[3];
float gyro[3],gyro_zero[3],gyro_buff[3];
float temp,cnt;
float yaw[3],yaw_cnt,yaw_diff;
float pitch;
float roll;
float q[4];
float xn[4],yn[4],zn[4],MotionAccel_b[4],AccelLPF,MotionAccel_n[4];
}bucket_imu;

typedef struct
{
	fp32 k_ff_1;			//一阶差分环节系数
	fp32 k_ff_2;			//二阶差分环节系数
	
	fp32 dif_1;				//一阶差分量
	fp32 dif_2;				//二阶差分量
	
	fp32 input_last;	//前一次的输入
	fp32 output;			//输出
	fp32 output_max;	//输出上限（绝对值）
	
	fp32 dbuf[2];			//（一阶）差分项，0：最新。1：上一次
}ff_type_def;

typedef struct{
  u8 tool_flag;
	int32_t raw_value;   					 //编码器不经处理的原始值
	int32_t last_raw_value;					 //上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;							 //两次编码器之间的差值
	int32_t temp_count;                      //计数用
	uint8_t buf_count;						 //滤波更新buf用
	int32_t ecd_bias;						 //初始编码器值	
	int32_t ecd_raw_rate;					 //通过编码器计算得到的速度原始值
	int32_t round_cnt;						 //圈数
	int32_t filter_rate;					 //速度
	int32_t electricity;
	float ecd_angle;						 //角度
	int32_t re_speed;
	float real_angle;
	float zero_angle;
	float relative_zero_angle;
	float ALL_angle;
//	void(*f_pid_reset)(struct PID *pid);
}dji_motor;

typedef struct{
float now_angle;
float now_angle_speed;
float now_torque;
float now_temp;
float total_angle;
uint16_t offset_angle;
}xiaomi_motor;



typedef struct{
	float set;
	float real;
	float Pout;
	float Iout;
	float Dout; 
	float Dout_last;
	float out;
	float interget_seperate;
	
	float error[3];
	float sum_error;
	
	float Kp;
	float Ki;
	float Kd;
	
	float PMAX;
	float IMAX;
	float DMAX;
	float OutMAX;	
  float sum_error_max;
	float real_out;
	
	u8 if_lp,if_is,if_isum;
	
  uint32_t DWT_CNT;
}PID;

typedef struct{
PID GIMBAL_PID[2];

}gimbal_pid;

typedef struct
{
int slope_in;
int slope_out;	
int slope_k;	
}slope;

typedef struct 
{
    float MatrixK[12];
    float StateFeedBack[6];
    float StateRef[6];
    float Output[2];
    float Outputmax;
}LQR;

typedef struct
{
    uint32_t motor_id                  : 8; 
    uint32_t data                      :16;
    uint32_t WTMFFF                    : 1;	  //飞坡模式标志位
    uint32_t gimbal_mode_change_flag   : 1;	  //云台切换自瞄和手瞄的标志位
    uint32_t chassic_mode              : 2;   //1 发射机构开炮 2 常规运动     3 小陀螺  
    uint32_t fire_flag                 : 1;	  //开火标志位
    uint32_t now_gimbal_station        : 3;   //1 底盘无力模式 2 遥控器控制模式 3 自动控制模式 
} __attribute__((packed)) EX_SEND_TO_CHASSIC;

typedef struct{
float pitch,yaw;
float pitch_output_offset,pitch_output_offset_area[3];
float rc_pitch_target_i,rc_yaw_target_i;
float fire_speed_,wait_for_cooling;
u8 fire_total_count;
EX_SEND_TO_CHASSIC Station_message;
float total_angle;
u8 hide_flag;
}Gimbal_hero;

typedef struct{
float gimbal_pit_sum,fimbal_yaw_sum;
float target_speed;
}PHC_HEITOU;

typedef __packed struct
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
	
	  float result;
} Ordinary_Least_Squares_t;

typedef struct{
	float C,C_IN;
	float K,K_IN;
	float ref; //初始目标值
	float error_eps;//误差下限
	float u_max;//输出最大值
	float J;//估计惯量
	float angle; //角度反馈，°
	float ang_vel;//角速度反馈，°/s
	float epsilon;

	float u;

	float error;
	float error_last;
	float dref;//目标值一阶导
	float ddref;//目标值二阶导
	float refl;//上一次的目标值

	float s;//滑模面
	
	float c_fuzzy[2],k_fuzzy[2];
}SMC_control;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//2023.12.11
#define hhv 0x800200
#define xyg 0x800400
#define cxk 0x800600
#define ldy 0x800800
#define xtt 0x800A00
#define zbw 0x800C00
#define hbb 0x800E00
#define wcy 0x801000
#define lzz 0x801200
#define ljy 0x801400
#define lwb 0x801600
#define lxy 0x801800
#define zjy 0x801A00
#define yjm 0x801C00
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Power_1_Pin GPIO_PIN_13
#define Power_1_GPIO_Port GPIOC
#define Power_2_Pin GPIO_PIN_4
#define Power_2_GPIO_Port GPIOC
#define SPI2_Gyro_CS_Pin GPIO_PIN_0
#define SPI2_Gyro_CS_GPIO_Port GPIOB
#define SPI2_Accel_CS_Pin GPIO_PIN_1
#define SPI2_Accel_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
