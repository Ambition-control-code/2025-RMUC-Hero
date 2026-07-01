#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "main.h"
//大疆电机数据结构体
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
	int16_t electricity;
	float ecd_angle;						 //角度
	int16_t re_speed;
	float real_angle;
	float zero_angle;
	float relative_zero_angle;
	float ALL_angle;
	int   zero_flag;
}DJI_motor;


typedef union{
u8 rc_station;
struct{
u8   zimiao_live:1;
u8	 direction:3;
u8 rc_sr : 2;
u8 rc_sl : 2;}rc;
}pack_recieve_gimbal;

void Solution_Message_IN_CAN3(u8 *a,int16_t ID);

//定义电机变量结构体数据
extern DJI_motor STEER_6020_I_MOTOR[4],WHEEL_3508_I_MOTOR[4];  
  /*dengtiao*/
//3_8      //2_7



//1_6      //0_5
extern uint16_t can_rx_data_num;

extern int16_t out_3508[4];
extern int16_t out_6020[4];

void Solution_Message_IN_CAN(u8 *a,int16_t ID);


void Motor_DUOLUN_PID_CTRL(void);
void chassic_control_task(void);
void Single_motor_power_test(void);
void Motor_PID_init(void);

void K1_K2_INIT(void);
void power_limit_duo_lun(float pow_max,int16_t *M_i_3508,int16_t*M_i_6020);

#endif



