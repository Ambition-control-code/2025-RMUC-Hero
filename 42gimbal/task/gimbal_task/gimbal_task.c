#include "stdlib.h"
#include "string.h"
#include "motor.h"
#include "rc.h"        
#include "pid_is.h"
#include "kalman.h"
#include "ulitility_tools.h"
#include "gimbal_task.h"
#include "refree_task.h"
#include "shoot_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "SolveTrajectory.h"
#include "ADRC_core.h"

extern extKalman_t yaw_RV;
extern RC rc;
extern bucket_imu imu;
extern dji_motor HERO_CLIP,SHOOT_L,SHOOT_R,SHOOT_H,balence_pitch,balence_yaw;
extern u8 live_all;
extern long int refree[9];
extern float zimiao_watch[4];

extern union{
VT13_data use_data;
u8  temp_data[21];
}VT13_USE;

LADRC_NUM LADRC_PITCH[3],LADRC_YAW[3];

PID PID_PITCH[2],PID_YAW[2],PID_V_P[2],PID_V_Y[2],P_S_L,P_S_R,PID_HERO_CLIP,PID_HERO_CLIP_ANGLE,P_S_H,motor_w_a,motor_w_s; 
Gimbal_all guard_gimbal;
ff_type_def yaw_speed,pitch_speed;

uint32_t dwt_cnt_controller_, dwt_cnt_observer;
float rc_p,rc_y,fire_speed_,PID_HERO_CLIP_ANGLE_sum,watch_gimbal_yaw,md_fire;
SMC_control yaw_smc;

void init_slow()
{
 PID_IS_Init(&PID_HERO_CLIP,12.85f,0.092f,0.0f,16384.0f,6000.0f,4000.0f,16384.0f,4000.0f,300.0f);
 PID_IS_Init(&PID_HERO_CLIP_ANGLE,150.0f,0.01f,100.0f,8000.0f,6000.0f,4000.0f,15000.0f,4000.0f,17.0f);	
}

void init_fast()
{
 PID_IS_Init(&PID_HERO_CLIP,12.85f,0.092f,0.0f,16384.0f,6000.0f,4000.0f,16384.0f,4000.0f,300.0f);
 PID_IS_Init(&PID_HERO_CLIP_ANGLE,280.0f,0.01f,100.0f,8000.0f,6000.0f,4000.0f,15000.0f,4000.0f,17.0f);
}

void gimbal_init()
{	
 LADRC_Init(&LADRC_PITCH[0],0.001,400,38,400,0.7);
 LADRC_Init(&LADRC_YAW[0],0.001,400,22,410,0.85);
	
 SMC_init(&yaw_smc,12.5f, 240, 0, 0.002, 25000, 1.8, 0.5);
	//SMC YawSMC(20, 120, 0, 0.001, 25000, 0.8, 0.5); //这是一个yaw轴参考电机参数
	
// PID_IS_Init(&PID_PITCH[0],200.85f,0.13f,0.0f,24000.0f,6000.0f,4000.0f,25000.0f,6000.0f,6000.0f);
	
 LADRC_Init(&LADRC_PITCH[1],0.001,500,70,400,0.8 );
 LADRC_Init(&LADRC_YAW[1]  ,0.001,580,58,390,0.75 );
	
 LADRC_Init(&LADRC_PITCH[2],0.001,400,100,400,0.8);
 LADRC_Init(&LADRC_YAW[2],0.001,600,60,350,0.75);
	
 PID_IS_Init(&P_S_L,18.85f,0.02f,0.0f,16000.0f,6000.0f,4000.0f,16384.0f,6000.0f,6000.0f);
 PID_IS_Init(&P_S_R,18.85f,0.02f,0.0f,16000.0f,6000.0f,4000.0f,16384.0f,6000.0f,6000.0f);	
 PID_IS_Init(&P_S_H,18.85f,0.02f,0.0f,16000.0f,6000.0f,4000.0f,16384.0f,6000.0f,6000.0f);
	
 PID_IS_Init(&motor_w_a,100,0.01,0.0f,16000.0f,15000.0f,4000.0f,16384.0f,10000.0f,4000);	
 PID_IS_Init(&motor_w_s,8.85,0.084,0.0f,16000.0f,15000.0f,4000.0f,16384.0f,10000.0f,4000);	
	
 init_slow();
}

float now_yaw,last_yaw;
u8 zimiao_count;
float diff;

void gimbal_output_calculate(void)
{
	if(live_all==0)
	{
		guard_gimbal.zimiao_use=0;	
		LADRC_PITCH[0].z1=0;
		LADRC_PITCH[0].z2=0;
		LADRC_PITCH[0].z3=0;
		LADRC_PITCH[1].z1=0;
		LADRC_PITCH[1].z2=0;
		LADRC_PITCH[1].z3=0;
		LADRC_YAW[0].z1=0;
		LADRC_YAW[0].z2=0;
		LADRC_YAW[0].z3=0;
		LADRC_YAW[1].z1=0;
		LADRC_YAW[1].z2=0;
		LADRC_YAW[1].z3=0;
  }
	else if(((rc.sl==1)||(rc.mouse_r)||(VT13_USE.use_data.fn_1==1)||(VT13_USE.use_data.mouse_right==1))&&(zimiao_watch[2]!=-1)&&(zimiao_watch[2]!=0))
	{	
		guard_gimbal.pitch_zimiao=-zimiao_watch[0];
		
		now_yaw=zimiao_watch[1];		

		guard_gimbal.yaw_zimiao=-(now_yaw+imu.yaw_cnt*360.0f);
		
		while((guard_gimbal.yaw_zimiao-(-imu.yaw[2]))> 300)
		{
		 guard_gimbal.yaw_zimiao-=360;
		}
		
		while((guard_gimbal.yaw_zimiao-(-imu.yaw[2]))<-300)
		{
		 guard_gimbal.yaw_zimiao+=360;
		}
		
		LADRC_Loop(&LADRC_PITCH[1],guard_gimbal.pitch_zimiao,imu.pitch,25000);
		LADRC_Loop(&LADRC_YAW[1],guard_gimbal.yaw_zimiao,-imu.yaw[2],25000);	
		
		rc.mouse_l=zimiao_watch[3];

		guard_gimbal.pitch_shoucao=guard_gimbal.pitch_zimiao;
		guard_gimbal.yaw_shoucao  =guard_gimbal.yaw_zimiao;				
			
		guard_gimbal.zimiao_use=1;
  }	
	else 
	{
		static int huanchong;
		
		float beishu,mousebeishu;
		
		if(VT13_USE.use_data.mode_sw==0)
		{
			beishu=0.00008,mousebeishu=0.2;
		}
		else
		{
		  beishu=0.0009,mousebeishu=2;		
		}
		
		guard_gimbal.pitch_shoucao+=rc.L_y*0.0012+(rc.mouse_y+VT13_USE.use_data.mouse_y*mousebeishu)/ 800.0f+refree[4]*(VT13_USE.use_data.ch_2-1024)*beishu;
		guard_gimbal.yaw_shoucao  +=rc.L_x*0.0012+(rc.mouse_x+VT13_USE.use_data.mouse_x*mousebeishu)/ 800.0f+refree[4]*(VT13_USE.use_data.ch_3-1024)*beishu;		
		
	  guard_gimbal.pitch_shoucao > 41  ? guard_gimbal.pitch_shoucao = 41 : 1;
	  guard_gimbal.pitch_shoucao < -21 ? guard_gimbal.pitch_shoucao = -21 : 1;
		
		if(((rc.F||VT13_USE.use_data.F)==1)&&(huanchong==1))
		{
		 guard_gimbal.yaw_shoucao+=180;
		 huanchong=0;
		}
		else if((rc.F||VT13_USE.use_data.F)==0)
		{
		 huanchong=1;
		}
		
		LADRC_Loop(&LADRC_PITCH[0],guard_gimbal.pitch_shoucao,imu.pitch,25000);
//		LADRC_Loop(&LADRC_YAW[0],guard_gimbal.yaw_shoucao,-imu.yaw[2],25000);	
		
		LADRC_YAW[0].u=SMC_Tick(&yaw_smc,guard_gimbal.yaw_shoucao,-imu.yaw[2],-imu.gyro_buff[2]*57.3f);
		
		guard_gimbal.zimiao_use=0;
  }	
}

int16_t output_a,output_b;
long long int count_creat;

void gimbal_task()   
{
	gimbal_init();
	while(1)
	{		
		refree[0]++;		

		gimbal_output_calculate();
		
		if(live_all)
		{
			if(guard_gimbal.zimiao_use==1)
			{
				PHC_GIMBAL((int16_t)LADRC_PITCH[1].u,(int16_t)LADRC_YAW[1].u);			
			}
			else
			{
				PHC_GIMBAL((int16_t)LADRC_PITCH[0].u,(int16_t)LADRC_YAW[0].u);
			}
		}
		else
		{
			PHC_GIMBAL((int16_t)0,(int16_t)0);	
		}
					
    if(rc.sl==3){init_fast();}		
    else if(rc.sl==1){init_slow();}		
		shoot_task();

		if(live_all)
		{
			gimbal_to_chassis_task();
		}
				
		vTaskDelay(2);
	}
}

