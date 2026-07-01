#ifndef __U_I_PID_H
#define __U_I_PID_H

#include "main.h"

typedef struct{
	float set;
	float real;
	float Pout;
	float Iout;
	float Dout;
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
	float OMAX;
	float OMIN;
	float OutMAX;	
  float sum_error_max;
	float real_out;
}PID;


extern PID CAP_charge_I; //电容充电电流环  //输入设定电流，调节实际pwm
extern PID CAP_charge_V; 
extern PID CAP_charge_P;
extern PID CAP_buffer_P; //电容缓冲能量环  //输入设定功率，调节实际功率

extern PID POW_charge_I; //电容放电电流环  //输入设定电流，调节实际pwm
extern PID POW_charge_V; 

extern PID PID_3508_speed_motor[4];
extern PID PID_6020_speed_motor[4];
extern PID PID_6020_angle_motor[4];

typedef struct FLT
{
    float    Fdata[3];
    uint16_t U16data[3];
}filter;


extern filter FLT_U_CAP;
extern filter FLT_I_CAP;
extern filter FLT_U_POW;
extern filter FLT_I_POW;//电压电源低通滤波器变量

extern float set_P;
extern float set_I;
extern float set_U;
extern uint8_t protection_mode;

void pid_init(PID *pid,float kp,float ki,float kd, float omax,float omin, float pmax, float imax, float dmax);
float  PID_Position(PID *Pid,float real,float set);
float  PID_Position_V(PID *Pid,float real,float set,float move_ment);
float  PID_Position_6020(PID *Pid,float real,float set,int16_t *M3508V_OFFSET);
float  PID_Incremental(PID *Pid,float real,float set);
float  PID_Position_ANGLE(PID *Pid,float real,float set);
float  PID_Incremental_lite(PID *Pid,float real,float set);
float  MAKE_ANGLE_TO_0_360(float angle);
float  FLT_COM(filter *FLT,float in,float arg);
void PID_CTRL(void);
//void CAP_I_CTRL(void);
//void CAP_U_CTRL(void);
void CAP_P_CTRL(void);
void CAP_U_I_CTRL(void);
void CAP_P_SET(void);
void BUFFER_ENERGY_CTRL(void);
void U_I_CTRL_10000HZ(void);
uint16_t HRTIM_PWM(uint16_t PWM);
float  PID_Position_DP(PID *Pid,float real,float set);
float  PID_Position_DPYAW(PID *Pid,float real,float set);


void  PID_IS_Init(PID* Pid,float kp,float ki,float kd,float pmax,float imax,float dmax,float outmax,float sum_error_max,float interget_seperate);

#endif
