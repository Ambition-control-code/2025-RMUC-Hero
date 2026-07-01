#include "motor.h"
#include "math.h"

float arctan;
float X;
float Y;
float W;
float V_BEI=5.0f;
//电机控制输出值
int16_t M_6020_OUT_I[4];
int16_t M_3508_OUT_I[4];

int16_t M_3508_V_OFFSET[4];
float   M3508_V_SET[4];

//每个电机单独解算出来的X,Y分量
float Motor_out_angle[4];

float Motor_vector_x[4];
float Motor_vector_y[4];

float Motor_sin_y[4];
float Motor_cos_x[4];

extern DJI_motor STEER_6020_I_MOTOR[4],WHEEL_3508_I_MOTOR[4],YAW_6020,HERO_CLIP; 
extern RC rc;


//舵轮相对于车身的坐标系偏移角度
#define OFFSET_Angle_6020_0x205  135
#define OFFSET_Angle_6020_0x206  45
#define OFFSET_Angle_6020_0x207  -135
#define OFFSET_Angle_6020_0x208  -45

void Send_6020_I_1FE(int16_t i1,int16_t i2,int16_t i3,int16_t i4)
{	
	uint8_t DATA_TX[8];
	
	DATA_TX[0] = i1 >> 8;
	DATA_TX[1] = i1;
	DATA_TX[2] = i2 >> 8;
	DATA_TX[3] = i2;
	DATA_TX[4] = i3 >> 8;
	DATA_TX[5] = i3;
	DATA_TX[6] = i4 >> 8;
	DATA_TX[7] = i4;
	
	fdcan_transmit(&hfdcan1, &TxHeader1,0x1FE, FDCAN_DLC_BYTES_8, DATA_TX);	
	//fdcanx_send_data(&hfdcan1,0x1FE,DATA_TX,8);
}

void Send_3508_I_200(int16_t i1,int16_t i2,int16_t i3,int16_t i4)
{	
	uint8_t DATA_TX[8];
	
	DATA_TX[0] = i1 >> 8;
	DATA_TX[1] = i1;
	DATA_TX[2] = i2 >> 8;
	DATA_TX[3] = i2;
	DATA_TX[4] = i3 >> 8;
	DATA_TX[5] = i3;
	DATA_TX[6] = i4 >> 8;
	DATA_TX[7] = i4;
	
	fdcan_transmit(&hfdcan1, &TxHeader1,0x200, FDCAN_DLC_BYTES_8, DATA_TX);	
	//fdcanx_send_data(&hfdcan1,0x200,DATA_TX,8);
}

void Motor_PID_init(void)
{
//	 //6020速度环初始化
//   //pid_init(PID *pid,               float kp,float ki,float kd, float omax,float omin, float pmax, float imax, float dmax)
//	 pid_init(&PID_6020_speed_motor[0],8.85f  ,0.01f    ,0.0f   ,16384.0f     ,-16384.0f  ,16000.0f,  6000.0f,   4000.0f); 
//	 pid_init(&PID_6020_speed_motor[1],8.85f  ,0.01f    ,0.0f   ,16384.0f     ,-16384.0f  ,16000.0f,  6000.0f,   4000.0f); 
//	 pid_init(&PID_6020_speed_motor[2],8.85f  ,0.01f    ,0.0f   ,16384.0f     ,-16384.0f  ,16000.0f,  6000.0f,   4000.0f); 	 
//	 pid_init(&PID_6020_speed_motor[3],8.85f  ,0.01f    ,0.0f   ,16384.0f     ,-16384.0f  ,16000.0f,  6000.0f,   4000.0f); 
//	
//	 //6020角度环初始化
//	
//	 pid_init(&PID_6020_angle_motor[0],32.85f ,0.01f     ,6.0f  ,600.0f       ,-600.0f    ,500.0f  ,60.0f    ,400.0f); 
//	 pid_init(&PID_6020_angle_motor[1],32.85f ,0.01f     ,6.0f  ,600.0f       ,-600.0f    ,500.0f  ,60.0f    ,400.0f); 
//	 pid_init(&PID_6020_angle_motor[2],32.85f ,0.01f     ,6.0f  ,600.0f       ,-600.0f    ,500.0f  ,60.0f    ,400.0f);  
//	 pid_init(&PID_6020_angle_motor[3],32.85f ,0.01f     ,6.0f  ,600.0f       ,-600.0f    ,500.0f  ,60.0f    ,400.0f); 		
//	
	 //3508速度环初始化

	 pid_init(&PID_3508_speed_motor[0],12.85f ,0.00f     ,0.1  ,16000.0f       ,-16000.0f    ,16000.0f ,6000.0f ,6000.0f); 
	 pid_init(&PID_3508_speed_motor[1],12.85f ,0.00f     ,0.1f  ,16000.0f       ,-16000.0f    ,16000.0f ,6000.0f ,6000.0f); 
	 pid_init(&PID_3508_speed_motor[2],12.85f ,0.00f     ,0.1f  ,16000.0f       ,-16000.0f    ,16000.0f ,6000.0f ,6000.0f);  
	 pid_init(&PID_3508_speed_motor[3],12.85f ,0.00f     ,0.1f  ,16000.0f       ,-16000.0f    ,16000.0f ,6000.0f ,6000.0f); 
	
	 //6020换向标志处初始化
	
	 M_3508_V_OFFSET[0]=1;
	 M_3508_V_OFFSET[1]=1;
	 M_3508_V_OFFSET[2]=1;
	 M_3508_V_OFFSET[3]=1;
	 
	 //电机K1 K2参数初始化
	 K1_K2_INIT();
}



void Motor_DUOLUN_PID_CTRL(void)
{
	//测算车头相对于底盘的偏转角通过角度矩阵变换映射到底盘
	
	
	//测算整体的车辆X Y W 横移+前进+横滚
	X=Remote_Ctrl.rc.ch[0]*V_BEI;
  Y=Remote_Ctrl.rc.ch[1]*V_BEI;
  W=Remote_Ctrl.rc.ch[2]*V_BEI;
	
	//通过sincos计算每个轮子w在x y方向的力分量在叠加到整车输出x y力分量中
	//分解
	Calculate_Float_Sin_Cos(-135.0f,&Motor_sin_y[0],&Motor_cos_x[0]);
	Calculate_Float_Sin_Cos(135.0f,&Motor_sin_y[1],&Motor_cos_x[1]);
	Calculate_Float_Sin_Cos(-45.0f,&Motor_sin_y[2],&Motor_cos_x[2]);
	Calculate_Float_Sin_Cos(45.0f,&Motor_sin_y[3],&Motor_cos_x[3]);
	//叠加
	Motor_vector_x[0]=X+Motor_cos_x[0]*W;
	Motor_vector_y[0]=Y+Motor_sin_y[0]*W;
	
	Motor_vector_x[1]=X+Motor_cos_x[1]*W;
	Motor_vector_y[1]=Y+Motor_sin_y[1]*W;
	
	Motor_vector_x[2]=X+Motor_cos_x[2]*W;
	Motor_vector_y[2]=Y+Motor_sin_y[2]*W;
	
	Motor_vector_x[3]=X+Motor_cos_x[3]*W;
	Motor_vector_y[3]=Y+Motor_sin_y[3]*W;
	//反tan求角
	Calculate_Float_arctan_all(Motor_vector_x[0],Motor_vector_y[0],&Motor_out_angle[0]);
	Calculate_Float_arctan_all(Motor_vector_x[1],Motor_vector_y[1],&Motor_out_angle[1]);
	Calculate_Float_arctan_all(Motor_vector_x[2],Motor_vector_y[2],&Motor_out_angle[2]);
	Calculate_Float_arctan_all(Motor_vector_x[3],Motor_vector_y[3],&Motor_out_angle[3]);
	//平方根求速度
  M3508_V_SET[0]= InvSqrt(Motor_vector_x[0]*Motor_vector_x[0]+Motor_vector_y[0]*Motor_vector_y[0]);
	M3508_V_SET[1]= InvSqrt(Motor_vector_x[0]*Motor_vector_x[0]+Motor_vector_y[0]*Motor_vector_y[0]);
	M3508_V_SET[2]= InvSqrt(Motor_vector_x[0]*Motor_vector_x[0]+Motor_vector_y[0]*Motor_vector_y[0]);
	M3508_V_SET[3]= InvSqrt(Motor_vector_x[0]*Motor_vector_x[0]+Motor_vector_y[0]*Motor_vector_y[0]);
	
	//6020pid 角度
	if(X==0&&Y==0&&W==0)//没事立正
	{
	  PID_Position_6020(&PID_6020_angle_motor[0],STEER_6020_I_MOTOR[0].real_angle-OFFSET_Angle_6020_0x205,-45,&M_3508_V_OFFSET[0]);
	  PID_Position_6020(&PID_6020_angle_motor[1],STEER_6020_I_MOTOR[1].real_angle-OFFSET_Angle_6020_0x206,45,&M_3508_V_OFFSET[1]);
	  PID_Position_6020(&PID_6020_angle_motor[2],STEER_6020_I_MOTOR[2].real_angle-OFFSET_Angle_6020_0x207,45,&M_3508_V_OFFSET[2]);
	  PID_Position_6020(&PID_6020_angle_motor[3],STEER_6020_I_MOTOR[3].real_angle-OFFSET_Angle_6020_0x208,-45,&M_3508_V_OFFSET[3]);		
		
//		M3508_V_SET[0]= -Remote_Ctrl.rc.ch[2]*V_BEI;
//	  M3508_V_SET[1]= -Remote_Ctrl.rc.ch[2]*V_BEI;
//	  M3508_V_SET[2]= Remote_Ctrl.rc.ch[2]*V_BEI;
//	  M3508_V_SET[3]= Remote_Ctrl.rc.ch[2]*V_BEI;
	}
	else
	{		
		PID_Position_6020(&PID_6020_angle_motor[0],STEER_6020_I_MOTOR[0].real_angle-OFFSET_Angle_6020_0x205,-Motor_out_angle[0],&M_3508_V_OFFSET[0]);
		PID_Position_6020(&PID_6020_angle_motor[1],STEER_6020_I_MOTOR[1].real_angle-OFFSET_Angle_6020_0x206,-Motor_out_angle[1],&M_3508_V_OFFSET[1]);
		PID_Position_6020(&PID_6020_angle_motor[2],STEER_6020_I_MOTOR[2].real_angle-OFFSET_Angle_6020_0x207,-Motor_out_angle[2],&M_3508_V_OFFSET[2]);
		PID_Position_6020(&PID_6020_angle_motor[3],STEER_6020_I_MOTOR[3].real_angle-OFFSET_Angle_6020_0x208,-Motor_out_angle[3],&M_3508_V_OFFSET[3]);		
	}
  //6020pid 速度
	M_6020_OUT_I[0]=PID_Position(&PID_6020_speed_motor[0],STEER_6020_I_MOTOR[0].re_speed,PID_6020_angle_motor[0].out);
	M_6020_OUT_I[1]=PID_Position(&PID_6020_speed_motor[1],STEER_6020_I_MOTOR[1].re_speed,PID_6020_angle_motor[1].out);
	M_6020_OUT_I[2]=PID_Position(&PID_6020_speed_motor[2],STEER_6020_I_MOTOR[2].re_speed,PID_6020_angle_motor[2].out);
	M_6020_OUT_I[3]=PID_Position(&PID_6020_speed_motor[3],STEER_6020_I_MOTOR[3].re_speed,PID_6020_angle_motor[3].out);		
	//3508pid 速度
	M_3508_OUT_I[0]=PID_Position(&PID_3508_speed_motor[0],WHEEL_3508_I_MOTOR[0].re_speed,M3508_V_SET[0]*M_3508_V_OFFSET[0]*(-1));
	M_3508_OUT_I[1]=PID_Position(&PID_3508_speed_motor[1],WHEEL_3508_I_MOTOR[1].re_speed,M3508_V_SET[1]*M_3508_V_OFFSET[1]*(-1));
	M_3508_OUT_I[2]=PID_Position(&PID_3508_speed_motor[2],WHEEL_3508_I_MOTOR[2].re_speed,M3508_V_SET[2]*M_3508_V_OFFSET[2]);
	M_3508_OUT_I[3]=PID_Position(&PID_3508_speed_motor[3],WHEEL_3508_I_MOTOR[3].re_speed,M3508_V_SET[3]*M_3508_V_OFFSET[3]);
	
	power_limit_duo_lun(80,M_3508_OUT_I,M_6020_OUT_I);
	
	Send_6020_I_1FE(out_6020[0],out_6020[1],out_6020[2],out_6020[3]);
	Send_3508_I_200(out_3508[0],out_3508[1],out_3508[2],out_3508[3]);
	//can发送电流值
	//Send_6020_I_1FE(M_6020_OUT_I[0],M_6020_OUT_I[1],M_6020_OUT_I[2],M_6020_OUT_I[3]);
	//Send_3508_I_200(M_3508_OUT_I[0],M_3508_OUT_I[1],M_3508_OUT_I[2],M_3508_OUT_I[3]);
	
}



void Motor_Steer_wheel_PID_CTRL(void)
{
	X=Remote_Ctrl.rc.ch[0]*V_BEI;
  Y=Remote_Ctrl.rc.ch[1]*V_BEI;
	Calculate_Float_arctan_all(X,Y,&arctan);
	
	
  M3508_V_SET[0]= InvSqrt(X*X+Y*Y);
	M3508_V_SET[1]= InvSqrt(X*X+Y*Y);
	M3508_V_SET[2]= InvSqrt(X*X+Y*Y);
	M3508_V_SET[3]= InvSqrt(X*X+Y*Y);
	
	if(X==0&&Y==0)
	{
	  PID_Position_6020(&PID_6020_angle_motor[0],STEER_6020_I_MOTOR[0].real_angle-OFFSET_Angle_6020_0x205,-45,&M_3508_V_OFFSET[0]);
	  PID_Position_6020(&PID_6020_angle_motor[1],STEER_6020_I_MOTOR[1].real_angle-OFFSET_Angle_6020_0x206,45,&M_3508_V_OFFSET[1]);
	  PID_Position_6020(&PID_6020_angle_motor[2],STEER_6020_I_MOTOR[2].real_angle-OFFSET_Angle_6020_0x207,45,&M_3508_V_OFFSET[2]);
	  PID_Position_6020(&PID_6020_angle_motor[3],STEER_6020_I_MOTOR[3].real_angle-OFFSET_Angle_6020_0x208,-45,&M_3508_V_OFFSET[3]);		
		
		M3508_V_SET[0]= -Remote_Ctrl.rc.ch[2]*V_BEI;
	  M3508_V_SET[1]= -Remote_Ctrl.rc.ch[2]*V_BEI;
	  M3508_V_SET[2]= Remote_Ctrl.rc.ch[2]*V_BEI;
	  M3508_V_SET[3]= Remote_Ctrl.rc.ch[2]*V_BEI;
	}
	else
	{		
		PID_Position_6020(&PID_6020_angle_motor[0],STEER_6020_I_MOTOR[0].real_angle-OFFSET_Angle_6020_0x205,-arctan,&M_3508_V_OFFSET[0]);
		PID_Position_6020(&PID_6020_angle_motor[1],STEER_6020_I_MOTOR[1].real_angle-OFFSET_Angle_6020_0x206,-arctan,&M_3508_V_OFFSET[1]);
		PID_Position_6020(&PID_6020_angle_motor[2],STEER_6020_I_MOTOR[2].real_angle-OFFSET_Angle_6020_0x207,-arctan,&M_3508_V_OFFSET[2]);
		PID_Position_6020(&PID_6020_angle_motor[3],STEER_6020_I_MOTOR[3].real_angle-OFFSET_Angle_6020_0x208,-arctan,&M_3508_V_OFFSET[3]);		
	}

	M_6020_OUT_I[0]=PID_Position(&PID_6020_speed_motor[0],STEER_6020_I_MOTOR[0].re_speed,PID_6020_angle_motor[0].out);
	M_6020_OUT_I[1]=PID_Position(&PID_6020_speed_motor[1],STEER_6020_I_MOTOR[1].re_speed,PID_6020_angle_motor[1].out);
	M_6020_OUT_I[2]=PID_Position(&PID_6020_speed_motor[2],STEER_6020_I_MOTOR[2].re_speed,PID_6020_angle_motor[2].out);
	M_6020_OUT_I[3]=PID_Position(&PID_6020_speed_motor[3],STEER_6020_I_MOTOR[3].re_speed,PID_6020_angle_motor[3].out);		
	
	M_3508_OUT_I[0]=PID_Position(&PID_3508_speed_motor[0],WHEEL_3508_I_MOTOR[0].re_speed,M3508_V_SET[0]*M_3508_V_OFFSET[0]*(-1));
	M_3508_OUT_I[1]=PID_Position(&PID_3508_speed_motor[1],WHEEL_3508_I_MOTOR[1].re_speed,M3508_V_SET[1]*M_3508_V_OFFSET[1]*(-1));
	M_3508_OUT_I[2]=PID_Position(&PID_3508_speed_motor[2],WHEEL_3508_I_MOTOR[2].re_speed,M3508_V_SET[2]*M_3508_V_OFFSET[2]);
	M_3508_OUT_I[3]=PID_Position(&PID_3508_speed_motor[3],WHEEL_3508_I_MOTOR[3].re_speed,M3508_V_SET[3]*M_3508_V_OFFSET[3]);
	
	power_limit_duo_lun(40,M_3508_OUT_I,M_6020_OUT_I);
	
	Send_6020_I_1FE(out_6020[0],out_6020[1],out_6020[2],out_6020[3]);
	Send_3508_I_200(out_3508[0],out_3508[1],out_3508[2],out_3508[3]);
	
	//Send_6020_I_1FE(M_6020_OUT_I[0],M_6020_OUT_I[1],M_6020_OUT_I[2],M_6020_OUT_I[3]);
	//Send_3508_I_200(M_3508_OUT_I[0],M_3508_OUT_I[1],M_3508_OUT_I[2],M_3508_OUT_I[3]);
	
}

//单个电机功率测试使用
void Single_motor_power_test(void)
{
	
	//M_6020_OUT_I[0]=PID_Position(&PID_6020_speed_motor[0],STEER_6020_I_MOTOR[0].re_speed,Remote_Ctrl.rc.ch[0]/1.0f);
	M_3508_OUT_I[0]=PID_Position(&PID_3508_speed_motor[0],WHEEL_3508_I_MOTOR[0].re_speed,Remote_Ctrl.rc.ch[0]*10);
	power_limit_duo_lun(80,M_3508_OUT_I,M_6020_OUT_I);
	
	Send_6020_I_1FE(out_6020[0],out_6020[1],out_6020[2],out_6020[3]);
	Send_3508_I_200(out_3508[0],out_3508[1],out_3508[2],out_3508[3]);
	
	Send_6020_I_1FE(M_6020_OUT_I[0],M_6020_OUT_I[1],M_6020_OUT_I[2],M_6020_OUT_I[3]);
	Send_3508_I_200(M_3508_OUT_I[0],M_3508_OUT_I[1],M_3508_OUT_I[2],M_3508_OUT_I[3]);
	
}

//什么也不干
void MOTOR_RUN_NULL()
{


}

//////////////////////////////////////////////////////////////

PID A_R,B_R,C_R,D_R,PID_YAW_FOLLOW,YAW_SPIN;
float all_kp=10.85f,all_ki=0.16,all_kd=0;
LOW_Pass_Filter lpx,lpy;

void pid_init_hero()
{
 LPF_Init(&lpx,0.1);
 LPF_Init(&lpy,0.1);
	
 PID_IS_Init(&A_R,all_kp,all_ki,0.0f,16000.0f,15000.0f,4000.0f,16384.0f,10000.0f,4000);
 PID_IS_Init(&B_R,all_kp,all_ki,0.0f,16000.0f,15000.0f,4000.0f,16384.0f,10000.0f,4000);
 PID_IS_Init(&C_R,all_kp,all_ki,0.0f,16000.0f,15000.0f,4000.0f,16384.0f,10000.0f,4000);	
 PID_IS_Init(&D_R,all_kp,all_ki,0.0f,16000.0f,15000.0f,4000.0f,16384.0f,10000.0f,4000);
		
	PID_IS_Init(&PID_YAW_FOLLOW,170,0.05,120,17000,2000,17000,32000,1000,20);
	PID_IS_Init(&YAW_SPIN,1.85f,0.01f,0,8000,2000,2000,8000,4000,2000);
}
#ifndef  mech_middle
#define mech_middle 46.109375
#endif

float find_shortest()
{
float shortest,relative_zero,relative_180,temp1,temp2;
relative_zero=mech_middle;
relative_180=mech_middle+180;
temp1=YAW_6020.ALL_angle-relative_zero;
temp2=YAW_6020.ALL_angle-relative_180;
while(temp1>180) {temp1-=360;}
while(temp1<-180) {temp1+=360;}
while(temp2>180) {temp2-=360;}
while(temp2<-180) {temp2+=360;}

return ((fabs(temp1) < fabs(temp2) ? temp1 : temp2));
}

float _speed_target[4];
float vx_set,vy_set,sin_yaw,cos_yaw;

void McNum_resolve()
{
	  static int turn_flag;
	  float x_target, y_target, r_target; 
		float use_x,use_y,speed_chassic;
		float display_position;
	
	  LPF_add(&lpx,rc.R_y);
	  LPF_add(&lpy,rc.R_x);

		use_x=-lpx.output*(9.7f+rc.Shift*4); 
		use_y=-lpy.output*(9.7f+rc.Shift*4);
		float aa = sqrt(use_x*use_x+use_y*use_y);
		x_target = aa < 8192 ? use_x : use_x*(8192/aa);
		x_target = aa >-8192 ? use_x : use_x*(8192/aa);
		y_target = aa < 8192 ? use_y : use_y*(8192/aa);
		y_target = aa >-8192 ? use_y : use_y*(8192/aa);
	
	  if(rc.Q==1)
		{
		turn_flag=1;
		}
		else if(rc.E==1)
		{
		turn_flag=0;
		}
	
		if(rc.live_over==0)
    {
		  for(int i=0;i<4;i++)
			{
				_speed_target[i]=0;
			}
		}
	  else if(rc.sl==1) 
    {
		 r_target=0;
		 rc.direction=0;
		}
		else if(rc.sl==2||(turn_flag)) 
    {
			rc.direction=1;
		float last_speed,temp_speed;
		temp_speed=0.25f*(WHEEL_3508_I_MOTOR[0].re_speed+WHEEL_3508_I_MOTOR[1].re_speed+WHEEL_3508_I_MOTOR[2].re_speed+WHEEL_3508_I_MOTOR[3].re_speed);
			
		if(robot.chassis_power_limit<75)
		{
		r_target=4000+rc.Shift*2000;
		}
		else if(robot.chassis_power_limit<90)
		{
		r_target=5000+rc.Shift*2000;		
		}
		else if(robot.chassis_power_limit<120)
		{
		r_target=6000+rc.Shift*2000;		
		}
		else
		{
		r_target=6500;
		}
		}
		else         
		{						
			PID_Position_DPYAW(&PID_YAW_FOLLOW,find_shortest(),0);
			
			if((x_target==0)||(y_target==0))
			{
				if(fabs(PID_YAW_FOLLOW.error[0])<0.5)
				{
					r_target = 0;			
				}
				else
				{
					r_target = PID_YAW_FOLLOW.out;			
				}
			}
			
		}
		
	  Calculate_Float_Sin_Cos((YAW_6020.ALL_angle+mech_middle),&sin_yaw,&cos_yaw);
    vx_set =  cos_yaw * x_target - sin_yaw * y_target;
    vy_set =  sin_yaw * x_target + cos_yaw * y_target;
		
		if(rc.Shift==0)
	 {
			vy_set > 5000 ? vy_set= 5000 : 1;
			vy_set <-5000 ? vy_set=-5000 : 1;	
			vy_set > 5000 ? vy_set= 5000 : 1;
			vy_set <-5000 ? vy_set=-5000 : 1;	
	 }
		else if(rc.Shift==1)
	 {
			if(vx_set> 8000) {vx_set= 8000;}
			if(vx_set<-8000) {vx_set=-8000;}
			if(vy_set> 8000) {vy_set= 8000;}
			if(vy_set<-8000) {vy_set=-8000;}
	 }

		_speed_target[0]= -vx_set+vy_set-r_target;
		_speed_target[1]= -vx_set-vy_set-r_target;
		_speed_target[2]=  vx_set-vy_set-r_target;
		_speed_target[3]=  vx_set+vy_set-r_target;
		
		for(int i;i<5;i++)
		{
		 if(_speed_target[i]>9000) {_speed_target[i]=9000;}
		 if(_speed_target[i]<-9000) {_speed_target[i]=-9000;}
		}
}

int16_t output_without_limit[4],piderror[4];

void pid_calculate_all(float expect[4])
{
		PID_Position_DP(&A_R,WHEEL_3508_I_MOTOR[0].re_speed,_speed_target[0]);
		PID_Position_DP(&B_R,WHEEL_3508_I_MOTOR[1].re_speed,_speed_target[1]);
		PID_Position_DP(&C_R,WHEEL_3508_I_MOTOR[2].re_speed,_speed_target[2]);
		PID_Position_DP(&D_R,WHEEL_3508_I_MOTOR[3].re_speed,_speed_target[3]);
	
		output_without_limit[0]= A_R.out;
		output_without_limit[1]= B_R.out;
		output_without_limit[2]= C_R.out;
		output_without_limit[3]= D_R.out;	
	
		piderror[0]= A_R.error[0];
		piderror[1]= B_R.error[0];
		piderror[2]= C_R.error[0];
		piderror[3]= D_R.error[0];	
}   
float out_limit[4];
/////////////////////////////////////////////////////////
void taskPowerControl(float *output);

void chassic_control_task()   
{
	static int i;
	if(i==0)
	{
	pid_init_hero();i++;
	}	
   	
	  McNum_resolve();
	
	  pid_calculate_all(_speed_target);

	  taskPowerControl(out_limit);

		Send_3508_I_200(out_limit[0],out_limit[1],out_limit[2],out_limit[3]);

}

/////////////////////////////////////////////////////////
uint16_t max_power_limit=0;
float chassis_max_power=0;
float input_power=0;           //电源功率（来自裁判系统）
float initial_give_power[4];   //PID计算的初始功率
float initial_total_power=0;   //总功率
float scaled_give_power[4];
PID powerControl_PID;

	
float chassis_power=0.0f;
float chassis_power_buffer=0.0f;
	
float CT=  1.99688994e-6f;         //力矩电流系数(20/16384)*(0.3)*(187/3591)/9.55
float k1 = 2.23e-07;             // k1
float k2 = 4.153e-07;            // k2
float constant = 1.181f;
float power_scale;
float ka,kb,kc;
float inside;
float temp;
float cmd;

extern  float  CAP_U;  //电容电压电流

/*******************************************************功率控制函数*************************************************************/
void taskPowerControl(float *output)
{
	chassis_max_power=0.0f;
	input_power=robot.chassis_power_limit;
  
	float display_power=120+(CAP_U-15)*100;
	display_power >150 ? display_power = 150 : 1;
	display_power < 80 ? display_power = 80 : 1;

	if((CAP_U>10.5)&&(rc.Shift==1))
	chassis_max_power=input_power+display_power;
	else
	chassis_max_power=input_power;
		
	initial_total_power=0.0f;
	for(uint8_t i=0;i<4;i++)
	{
		initial_give_power[i]=output_without_limit[i]*CT*WHEEL_3508_I_MOTOR[i].re_speed+
		                      k1*WHEEL_3508_I_MOTOR[i].re_speed*WHEEL_3508_I_MOTOR[i].re_speed+
		                      k2*output_without_limit[i]*output_without_limit[i]+constant;   //初始每个轮子的功率
		if (initial_give_power[i] < 0) 
			continue;
		initial_total_power += initial_give_power[i];                                                       //计算总功率
	}
	
	if (initial_total_power > chassis_max_power) //是否大于最大功率
	{
		//按电机分配功率
		power_scale = chassis_max_power / initial_total_power;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale; // 得到缩放功率
			if (scaled_give_power[i] < 0)
			{
				continue;
			}
			
			ka=k1;
			kb = CT * WHEEL_3508_I_MOTOR[i].re_speed;
			kc = k2 * WHEEL_3508_I_MOTOR[i].re_speed * WHEEL_3508_I_MOTOR[i].re_speed - scaled_give_power[i] + constant;
			inside = kb * kb - 4 * ka * kc;

			if (inside < 0)
			{
				continue;
			}
			else if (output_without_limit[i] > 0) //根据原始力矩方向选择计算公式
			{
				cmd=InvSqrt(inside);
				temp = (-kb + cmd) / (2 * ka);
				output[i]=(temp>16384?16384:temp);
			}
			else
			{
				cmd=InvSqrt(inside);
				temp = (-kb - cmd) / (2 * ka);
				output[i]=(temp<-16384?-16384:temp);
			}
		}
		return;
	}
	//若总功率未超过最大功率，则将原始速度环PID的值发给底盘
	for(uint8_t i=0;i<4;i++)
	{
		output[i]=output_without_limit[i];
	}
}




