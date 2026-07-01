#include "cmsis_os.h"
#include "stdlib.h"
#include "string.h"
#include "motor.h"
#include "rc.h"        
#include "pid_is.h"
#include "MI_Cyber_Gear.h"
#include "ulitility_tools.h"
#include "refree_task.h"
#include "bsp_dwt.h"
#include "gimbal_task.h"

long int refree[9];
u8 live_all;

extern Gimbal_all guard_gimbal;
extern RC rc;

int aim,count;
extern long long int count_creat;
extern u8 recieve_error;

JOYSTICK_TypeDef my_joystick;  //手柄键值结构体

u8 rps,all_rps;

void refree_task()   
{
	while(1)
	{
		if(refree[7]>0)
		{
			all_rps=refree[7];
			refree[7]=0;
			live_all=1;
			rps=refree[8];
			refree[8]=0;
			refree[4]=1;
		}
		else
		{
			live_all=0;refree[4]=0;}
		
		if(refree[5]>0)
		{
			live_all=1;
		}
		else
		{
			live_all=0;refree[4]=0;
		}
		
		if(recieve_error)
		{
			RC_restart(36);
			recieve_error=0;	
		}
		
//		AX_PS2_ScanKey(&my_joystick);
		
		count++;	
		vTaskDelay(100);

		}
//		if(rc.sl==1){
//		if(count==5)
//		{
//		count=0;
//		if(aim){guard_gimbal.yaw_zimiao=20;aim=0;}
//	  else{guard_gimbal.yaw_zimiao=-20;	aim=1;} 
//	  }
//		}
//		else
//		{
//		 guard_gimbal.yaw_zimiao=speed_fliter(20*sin(count_creat*0.01));
//		}
		
	  
}

//#define PI_mu 0.01745f 
//float power_limit_k = 1.0f;
//void power_limit(float pow_max, double k1, double k2)    //功率控制函数
//{
//	
//	float velo1, velo2;
//	velo1 = (float)CM1Encoder.re_speed) / 14.0f / 60.0f * 2.0f * PI;
//	velo2 = ((float)CM2Encoder.re_speed) / 14.0f / 60.0f * 2.0f * PI;
//	float cmd1, cmd2;
//	
//	cmd1 = ((float)out_incr[BR]) / 16384.0f * 20.0f * 3.0f / 8.0f;
//	cmd2 = ((float)out_incr[BL]) / 16384.0f * 20.0f * 3.0f / 8.0f;
//	
//	float sum_velo_cmd, sum_velo_cmd2;
//	sum_velo_cmd = fabs(velo1 * cmd1) + fabs(velo2 * cmd2);
//	sum_velo_cmd2 = pow(sum_velo_cmd, 2);
//	
//	float sum_cmd2, sum_velo2;
//	sum_cmd2 = pow(cmd1, 2) + pow(cmd2, 2);
//	sum_velo2 = pow(velo1, 2) + pow(velo2, 2);
//	
//	float mid_temp;
//	mid_temp = sum_velo_cmd2 - 4 * k1 * sum_cmd2 * (k2 * sum_velo2 - pow_max);
//	if (mid_temp > 0) {
//		power_limit_k = (-sum_velo_cmd + sqrt(mid_temp)) / (2 * k1 * sum_cmd2);
//	}
//	else {
//		power_limit_k = .0;
//	}
//	if (power_limit_k > 1) {
//		power_limit_k = 1;
//	}
//	out_limit[FR] = power_limit_k * out_incr[FR];
//	out_limit[FL] = power_limit_k * out_incr[FL];
//		
//}












