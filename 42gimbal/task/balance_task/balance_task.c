#include "cmsis_os.h"
#include "stdlib.h"
#include "string.h"
#include "motor.h"
#include "rc.h"        
#include "pid_is.h"
#include "MI_Cyber_Gear.h"
#include "ulitility_tools.h"
#include "balance_task.h"
#include "bsp_dwt.h"
#include "VMC_calc.h"
#include "lqr.h"




extern RC rc;
extern bucket_imu imu;
extern MI_Motor_s  MI_Motor,MI_Motor2; 
extern mi_leg_motor MI_LEFT,MI_RIGHT;
extern old_mi OMI_1_4,OMI_1_1,OMI_2_4,OMI_2_1;

//KalmanFilter_t kf;         
balance_leg leg_ambition;
vmc_leg_t VMC_left,VMC_right;
balance_usedata left_all_,right_all_;

uint32_t dwt_cnt_VMC;

float kp_of_split=20;
float kp_of_turn=2.7f;
float kd_of_turn=0.22f;
float kp_leg=1100;
float kp_leg_tp=-16.3f;
float feed_out_body=-20;
float max_of_F0=45;
float max_of_Tp=6;
float change_l0;

float limit(float in,float max)
{
		if(in>max){in=max;}
		if(in<-max){in=-max;}  
		return in;
}

float k_balance[12]={  -15.9121  , -1.5919   ,  -0.1583  ,  -2.5963  ,  8.0581  ,  0.8828 ,
                       45.0700   , 7.2497    ,   1.1174  ,  18.2339  , 10.3750  , -0.9955     };
											 
/**
 *    pitch沿x轴逆时针旋转为负
 *    ^ y            左轮     右轮       D A
 *    |              ∠1 |     |  A0   --O O--
 *    |_____ >  x        |-----|       \     /               /
 *                   ∠4 |     |  D0    \   /               /
 *    z 轴从屏幕向外                      O ->正方向       /    腿部摆角正方向        
 *    实际值：
 *          1.左右腿摆角正方向。
 *					2.陀螺仪pitch与腿部摆角极性。
 *					3.轮毂方向。
 *				  4.TP方向。
 */

float kp=3300;
void balance_task()   
{
	VMC_init(&VMC_left);
	VMC_init(&VMC_right);
	dwt_cnt_VMC=DWT_GetTimeline_s();
  while(1)
  {			
		static int i;		
		
	 VMC_left.dt=DWT_GetDeltaT(&dwt_cnt_VMC);
	 VMC_right.dt=VMC_left.dt;
		
   VMC_left.phi1=(57.841713)/57.29+OMI_1_1.pos;           //-55.2815247
   VMC_left.phi4=OMI_1_4.pos+102.584641/57.29;
		
	 VMC_calc_1_left(&VMC_left,&imu,VMC_left.dt);
		
   VMC_right.phi4=(-OMI_2_4.pos);
   VMC_right.phi1=((180+78.9182968)/57.29-OMI_2_1.pos);
		
		VMC_calc_1_right(&VMC_right,&imu,VMC_right.dt);
		
		float display_tp=kp_of_split*(VMC_left.alpha-VMC_right.alpha);//防劈叉，不然只要机体yaw上有环俩腿就会批

//    left_all_.pitch=imu.pitch/57.29;
//    left_all_.w_pitch=imu.gyro_buff[0];
//    right_all_.pitch=imu.pitch/57.29;
//    right_all_.w_pitch=imu.gyro_buff[0];
//		
//    Lqr_Calc(&VMC_left,&left_all_);		
//    Lqr_Calc(&VMC_right,&right_all_);			

    if(rc.sl==1)
		{
    VMC_left.Tp=k_balance[0]*(-VMC_left.theta)-k_balance[3]*leg_ambition.real_speed+imu.pitch/57.3*k_balance[4];//+k_balance[1]*VMC_left.d_theta;    //Tp
		if(VMC_left.Tp>max_of_Tp){VMC_left.Tp=max_of_Tp;}
		if(VMC_left.Tp<-max_of_Tp){VMC_left.Tp=-max_of_Tp;}
		
    VMC_right.Tp=k_balance[0]*(VMC_right.theta)+k_balance[3]*leg_ambition.real_speed-imu.pitch/57.3*k_balance[4];//+k_balance[1]*-VMC_left.d_theta;    //Tp
		if(VMC_right.Tp>max_of_Tp) {VMC_right.Tp=max_of_Tp;}
		if(VMC_right.Tp<-max_of_Tp){VMC_right.Tp=-max_of_Tp;}
	  }
	else
    {
    VMC_left.Tp=k_balance[0]*(-VMC_left.alpha);//Tp
		if(VMC_left.Tp>max_of_Tp){VMC_left.Tp=max_of_Tp;}
		if(VMC_left.Tp<-max_of_Tp){VMC_left.Tp=-max_of_Tp;}
		
    VMC_right.Tp=k_balance[0]*(VMC_right.alpha);//Tp
		if(VMC_right.Tp>max_of_Tp) {VMC_right.Tp=max_of_Tp;}
		if(VMC_right.Tp<-max_of_Tp){VMC_right.Tp=-max_of_Tp;}	
	  }
		
		VMC_left.F0=-feed_out_body*cosf(VMC_left.theta)-kp_leg*(VMC_left.L0-0.18f-0.02f);    //F0竖直
		if(VMC_left.F0>max_of_F0){VMC_left.F0=max_of_F0;}
		if(VMC_left.F0<-max_of_F0){VMC_left.F0=-max_of_F0;}  
		
    VMC_right.F0=feed_out_body*cosf(VMC_right.theta)+(kp_leg)*(VMC_right.L0-0.18f);    //F0竖直
		if(VMC_right.F0>max_of_F0) {VMC_right.F0=max_of_F0;}
		if(VMC_right.F0<-max_of_F0){VMC_right.F0=-max_of_F0;} 
		
   	VMC_calc_2(&VMC_left); 
   	VMC_calc_2(&VMC_right);	
				
		 Send_to_LMI(1,1,VMC_left.torque_set[0]);
		 Send_to_LMI(4,1,VMC_left.torque_set[1]);

		 Send_to_LMI(1,2,VMC_right.torque_set[0]);
		 Send_to_LMI(4,2,VMC_right.torque_set[1]);
//		
//		 Send_to_LMI(1,1,0);
//		 Send_to_LMI(4,1,0);

//		 Send_to_LMI(1,2,0);
//		 Send_to_LMI(4,2,0);
		
//////////轮毂部分																	
		 int wheel_mode;
		 if(VMC_left.L0>0.15||VMC_left.L0>0.15)
		 {
		   wheel_mode=1;
		 }
		 else
		 {
		   wheel_mode=0;
		 }
				
		static float yaw_target;

    yaw_target+=rc.R_x*0.0006;
				
		float display_yaw
			=(imu.yaw[2]-yaw_target)/57.3f*kp_of_turn+kd_of_turn*imu.gyro[2];		
		
		leg_ambition.expect_speed=rc.R_y/220;
		
		leg_ambition.theta=-VMC_left.theta*k_balance[6];
//		leg_ambition.d_theta=VMC_left.d_theta*k_balance[7]; 
		leg_ambition.output_l_speed=leg_ambition.expect_speed-leg_ambition.real_speed;
		leg_ambition.output_l_pitch=imu.pitch/57.3*k_balance[10];
		leg_ambition.output_l_gyro=imu.gyro_buff[0]*k_balance[11];
		
		leg_ambition.output_all_l[0]=leg_ambition.output_l_pitch+leg_ambition.output_l_gyro
		+leg_ambition.output_l_speed
		+leg_ambition.theta+leg_ambition.d_theta;
		leg_ambition.output_all_l[1]=leg_ambition.output_l_pitch+leg_ambition.output_l_gyro
		+leg_ambition.output_l_speed
		+leg_ambition.theta+leg_ambition.d_theta;
				
		if(rc.sl==1){
		MI_motor_TorqueControl(&MI_Motor2 , limit( leg_ambition.output_all_l[0]/3,6));  //left
		MI_motor_TorqueControl(&MI_Motor  , limit(-leg_ambition.output_all_l[1]/3,6));		
		}
		else{
		MI_motor_TorqueControl(&MI_Motor ,0);
		MI_motor_TorqueControl(&MI_Motor2,0);}	
		
	 vTaskDelay(3);
  }
}











