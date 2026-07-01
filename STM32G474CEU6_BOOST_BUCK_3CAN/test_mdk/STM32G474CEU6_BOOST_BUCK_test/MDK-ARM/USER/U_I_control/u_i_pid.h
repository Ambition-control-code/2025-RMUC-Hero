#ifndef __U_I_PID_H
#define __U_I_PID_H

#include "main.h"
//
//建议PWM上下限制
#define RER        23040
#define CMPPWM_MAX 22900
#define CMPPWM_MIN 100
#define CAP_MAX_V 22500
#define CAP_MIN_V 6000

typedef struct PIDset
{  
		 
     double PMAX;
		 double IMAX;
		 double DMAX;     //PID限幅
		
		 float OMAX;      //最终输限幅
	   float OMIN;
		
		 float Kp,Ki,Kd;  //pid参数
		
		 float out;       //计算输出值
		 
		 float Pout;
		 float Iout;
		 float Dout;      //三个参数分别计算出的输出值
		
     float error[3];     
		 float sum_error; 
		 
     float set;       //pid输入的设定值
     float real;      //当前的真实值
     	 	
}PID;


extern PID CAP_charge_I; //电容充电电流环  //输入设定电流，调节实际pwm
extern PID CAP_charge_V; 
extern PID CAP_charge_P;

extern PID POW_charge_I; //电容放电电流环  //输入设定电流，调节实际pwm
extern PID POW_charge_V; 


typedef struct FLT
{
    float    Fdata[3];
    uint16_t U16data[3];
}filter;

extern PID CAP_I_PID; //电容端电流环
extern PID CAP_U_PID; //电压端电压环

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
float  PID_Incremental(PID *Pid,float real,float set);
float  PID_Position_ANGLE(PID *Pid,float real,float set);
float  PID_Incremental_lite(PID *Pid,float real,float set);
float  FLT_COM(filter *FLT,float in,float arg);
void PID_CTRL(void);
void CAP_I_CTRL(void);
void CAP_U_CTRL(void);
void CAP_P_CTRL(void);
void CAP_U_I_CTRL(void);
void CAP_P_SET(void);
uint16_t HRTIM_PWM(uint16_t PWM);


#endif
