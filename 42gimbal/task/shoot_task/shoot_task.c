#include "stdlib.h"
#include "string.h"
#include "motor.h"
#include "rc.h"        
#include "pid_is.h"
#include "kalman.h"
#include "ulitility_tools.h"
#include "shoot_task.h"
#include "refree_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gimbal_task.h"

extern PID PID_PITCH[2],PID_YAW[2],PID_V_P[2],PID_V_Y[2],P_S_L,P_S_R,PID_HERO_CLIP,PID_HERO_CLIP_ANGLE,P_S_H,motor_w_a,motor_w_s;
extern Gimbal_all guard_gimbal;

extern RC rc;
extern bucket_imu imu;
extern dji_motor HERO_CLIP,SHOOT_L,SHOOT_R,SHOOT_H,HERO_WATCH;
extern u8 live_all;
extern long int refree[9];

extern union{
VT13_data use_data;
u8  temp_data[21];
}VT13_USE;

float angle_of_watch;

static int _last,_lastvt13;
int shoot;
int shoot_error_check,shoot_error_flag;

uint8_t refree_offline_judgement()         //判断裁判系统是否离线
{
	if(guard_gimbal.max_hot==0)
	{return 1;}
	else 
	return 0;
}

uint8_t dial_exception_handling(u8 flag)   //卡弹检测与处理
{
	if(fabs(PID_HERO_CLIP_ANGLE.error[0])>5)
	{
		 shoot_error_check++;
	}
	else 
	{
		 shoot_error_check=0;
     guard_gimbal.enable_dial=1;
	}      
	
	if(shoot_error_check>1200)
	{
	  shoot_error_check=1200;
	}
	
	static int i;
	if(!i)                                   //未出现未处理卡弹情况
	{
	 	if(shoot_error_check>1000)             //出现卡弹情况
		{
			if(guard_gimbal.Dial_behavior==3)    //若处于单发模式则取消当前发（只取消一次）
			guard_gimbal.PID_HERO_CLIP_ANGLE_sum=guard_gimbal.last_sum; 
			i=1;                                 //进入卡单待处理模式
			guard_gimbal.enable_dial=0;          //失能拨盘
			return 0;
		}
		else
		{
		 return flag;                          //正常返回拨盘行为
		}
	}
	else
	{	
		if((rc.Z==1)||(shoot_error_check==0))  //有手动反转处理或者关闭摩擦轮重新打开
		{
			i=0;
			shoot_error_check=0;                 //重新检测
			guard_gimbal.enable_dial=1;          //使能拨盘
		}
		else
		{
			guard_gimbal.enable_dial=0;          //失能拨盘
			return 0;
		}
  }
}

void fast_shoot_station()                  //单发快拨模式
{
	guard_gimbal.last_sum=guard_gimbal.PID_HERO_CLIP_ANGLE_sum;
	guard_gimbal.PID_HERO_CLIP_ANGLE_sum+=72; 
	rc.sr_change_flag=0;
	guard_gimbal.Dial_behavior=5;
	PID_IS_Init(&PID_HERO_CLIP,12.85f,0.092f,0.0f,16384.0f,6000.0f,4000.0f,16384.0f,4000.0f,300.0f);
	PID_IS_Init(&PID_HERO_CLIP_ANGLE,280.0f,0.01f,100.0f,8000.0f,6000.0f,4000.0f,15000.0f,4000.0f,17.0f);
}

void slow_shoot_station()                  //慢拨模式
{
	guard_gimbal.last_sum=guard_gimbal.PID_HERO_CLIP_ANGLE_sum;
	guard_gimbal.PID_HERO_CLIP_ANGLE_sum+=0.04; 	
	PID_IS_Init(&PID_HERO_CLIP,12.85f,0.092f,0.0f,16384.0f,6000.0f,4000.0f,16384.0f,4000.0f,300.0f);
	PID_IS_Init(&PID_HERO_CLIP_ANGLE,150.0f,0.01f,100.0f,8000.0f,6000.0f,4000.0f,15000.0f,4000.0f,17.0f);
}

void reverse_dial()                        //手控反转
{
	guard_gimbal.PID_HERO_CLIP_ANGLE_sum-=0.2;	
}


void shoot_task()   
{		
	if((rc.sr==1||rc.sr==3)||(VT13_USE.use_data.mode_sw==2||VT13_USE.use_data.mode_sw==0))
	{
		guard_gimbal.fire_speed=4950;      //5405 15.5   4500 12.0   4800 13.3
		
		guard_gimbal.enable_dial=1;
		
		if((_last-(rc.mouse_l||VT13_USE.use_data.mouse_left)==-1)||(_lastvt13-VT13_USE.use_data.trigger==-1))
		{
  		shoot=1;
		}
		else
		{
  		shoot=0;
		}
		
		_last=(rc.mouse_l||VT13_USE.use_data.mouse_left);
		_lastvt13=VT13_USE.use_data.trigger;
		
		if(((refree_offline_judgement())||(guard_gimbal.max_hot-guard_gimbal.now_hot>=60)||(rc.G)))
		//热量限制(G无视热量限制)||若无裁判系统连接则取消热量限制
		{
			if(rc.Z==1)                                                //反转模式
	    {
				guard_gimbal.Dial_behavior=1;
	    }
//			else if(rc.mouse_r==1)                                     //自动开火（尚未完成）
//			{
//			}
			else if((((rc.sr_change_flag)&&(rc.last_sr==1))||(shoot))) //单发模式
			{
				guard_gimbal.Dial_behavior=3;
			}
			else if(((rc.sr==3)&&(rc.trigger>3000))||(VT13_USE.use_data.fn_2==1))//慢拨模式
			{
				guard_gimbal.Dial_behavior=4;				
			}
			else
			{
				guard_gimbal.Dial_behavior=5;
			}
	  }
		
//		if(rc.G==0)//按下G时无视全部限制
//		guard_gimbal.Dial_behavior=dial_exception_handling(guard_gimbal.Dial_behavior);//是否卡弹
		//若进入卡弹待处理，则不再接受除Z及直接关闭摩擦轮外其他处理
		
		switch(guard_gimbal.Dial_behavior)
		{
			case 0:{}break;
			case 1:{reverse_dial();}break;
			case 2:{}break;
			case 3:{fast_shoot_station();}break;
			case 4:{slow_shoot_station();}break;
			default:  break;
		}		
	}
	else  
	{
		guard_gimbal.fire_speed=0;
		guard_gimbal.PID_HERO_CLIP_ANGLE_sum=HERO_CLIP.relative_zero_angle*27/19;
	}    
	
	if((VT13_USE.use_data.wheel!=1024)&&(VT13_USE.use_data.wheel>200)){
	angle_of_watch+=0.0004*(VT13_USE.use_data.wheel-1024);}
	
	PID_Position(&motor_w_a,HERO_WATCH.zero_angle,angle_of_watch);
	PID_Position(&motor_w_s,HERO_WATCH.re_speed           ,motor_w_a.out);

	PID_Position(&P_S_L,SHOOT_L.re_speed,-guard_gimbal.fire_speed*live_all);
	PID_Position(&P_S_R,SHOOT_R.re_speed, guard_gimbal.fire_speed*live_all); 
	PID_Position(&P_S_H,SHOOT_H.re_speed,-guard_gimbal.fire_speed*live_all); 
	
	PID_Position(&PID_HERO_CLIP_ANGLE,HERO_CLIP.relative_zero_angle*27/19,guard_gimbal.PID_HERO_CLIP_ANGLE_sum);
	PID_Position(&PID_HERO_CLIP      ,HERO_CLIP.re_speed           ,PID_HERO_CLIP_ANGLE.out);
	
	Motor_Set_Current2_0_4(P_S_R.out,P_S_L.out,P_S_H.out,0); //发送	
	Motor_Set_Current1_0_4(PID_HERO_CLIP.out*live_all*guard_gimbal.enable_dial,motor_w_s.out,0,0); //发送	
}











