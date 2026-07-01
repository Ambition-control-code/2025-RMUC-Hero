#include "motor.h"
#include "math.h"
int16_t out_3508[4];
int16_t out_6020[4];

float K1_3508[4];
float K2_3508[4];
float K1_6020[4];
float K2_6020[4];
//rpm——》rad 转速——弧度速度转换系数rad=rpm/()
#define  RPM_RAD  1.46607654f   //14.0f / 60.0f * 2.0f * PI
//Kt   力矩电流和力矩的转换系数
#define  Kt_3508_14  0.02142f   //0.3*187/3591   我们 1/14
#define  Kt_6020_1   0.741f      //0.3*187/3591  我们 1/14
//t_CMD   电流原始至和力矩缩放系数
#define T_CMD_6020   0.000135681152f //(3.0f/16384.0f  * Kt_6020_1)
#define T_CMD_3508   0.00002614746f  //(20.0f/16384.0f  * Kt_3508_14)
//将以上系数全部算好，得到缩放公式
//Pin=Ct*Icmd*w+k2*w*w+k1*icmd*icmd
//	float MP1,MP2,MP3,MP4;
//  float velo1, velo2, velo3, velo4;
//  float cmd1, cmd2, cmd3, cmd4;
//	float P_ALL;                 //实际总体功率
//void power_limit_2_0(float pow_max, float *k1, float *k2,  )
//{
//	
//	//float velo1, velo2, velo3, velo4;
//  //求电机旋转速度 原始转速值为RPM
//	velo1 = ((float)CM1Encoder.re_speed) ;
//	velo2 = ((float)CM2Encoder.re_speed) ;
//	velo3 = ((float)CM3Encoder.re_speed) ;
//	velo4 = ((float)CM4Encoder.re_speed) ;
//	
//	
//	//float cmd1, cmd2, cmd3, cmd4;
//	//求电机设定转矩
//	cmd1 = ((float)out_incr[BR])  *T_CMD;
//	cmd2 = ((float)out_incr[BL])  *T_CMD;
//	cmd3 = ((float)out_incr[FL])  *T_CMD;
//	cmd4 = ((float)out_incr[FR])  *T_CMD;
//	
//	
//	//float MP1,MP2,MP3,MP4;
//	//求电机设定输出功率//Pmax=K1*t*t+K2*w*w+w*t/9.55 
//	MP1=k1[0]*cmd1*cmd1+k2[0]*velo1*velo1+cmd1*velo1/9.55f;
//	MP2=k1[1]*cmd2*cmd2+k2[1]*velo2*velo2+cmd2*velo2/9.55f;
//	MP3=k1[2]*cmd3*cmd3+k2[2]*velo3*velo3+cmd3*velo3/9.55f;
//	MP4=k1[3]*cmd4*cmd4+k2[3]*velo4*velo4+cmd4*velo4/9.55f;
//	
//	//float P_ALL;                 //实际总体功率
//	float power_limit_k = 1.0f;  //功率缩放系数
//	
//	P_ALL=MP1+MP2+MP3+MP4;
//	power_limit_k=pow_max/P_ALL;
//	
//	float t1,t2; //转矩电流的两个解
//	
//	if(power_limit_k<=1&&power_limit_k>=0)
//	{
//		 MP1=power_limit_k*MP1;
//		 MP2=power_limit_k*MP2;
//		 MP3=power_limit_k*MP3;
//		 MP4=power_limit_k*MP4;
//	   if(MP1<=0)//单个电机负功不缩放
//     {
//		    out_limit[BR] =out_incr[BR];
//		 }	
//     else
//     {
//		    t1=DATA_T1(velo1,MP1,k1,k2);
//			  t2=DATA_T2(velo1,MP1,k1,k2);
//			  out_limit[BR]=return_real_cmd(t1,t2,chassis_3508_ID[0].error[0],velo1)/(T_CMD);
//		 }		

//	   if(MP2<=0)//单个电机负功不缩放
//     {
//		    out_limit[BL] =out_incr[BL];
//		 }	
//     else
//     {
//		   	t1=DATA_T1(velo2,MP2,k1,k2);
//			  t2=DATA_T2(velo2,MP2,k1,k2);
//			  out_limit[BL]=return_real_cmd(t1,t2,chassis_3508_ID[1].error[0],velo2)/(T_CMD);	 
//		 }

//	   if(MP3<=0)//单个电机负功不缩放
//     {
//		    out_limit[FL] =out_incr[FL];
//		 }	
//     else
//     {
//		    t1=DATA_T1(velo3,MP3,k1,k2);
//			  t2=DATA_T2(velo3,MP3,k1,k2);
//			  out_limit[FL]=return_real_cmd(t1,t2,chassis_3508_ID[2].error[0],velo3)/(T_CMD);
//		 }

//	   if(MP4<=0)//单个电机负功不缩放
//     {
//		    out_limit[FR] =out_incr[FR];
//		 }	
//     else
//     {
//			  t1=DATA_T1(velo4,MP4,k1,k2);
//			  t2=DATA_T2(velo4,MP4,k1,k2);
//			  out_limit[FR]=return_real_cmd(t1,t2,chassis_3508_ID[3].error[0],velo4)/(T_CMD); 
//		 }		 
//		
//		
//	
//	
//	}
//	else
//	{	//做负功或者没到Pmax，不用缩放
//		  power_limit_k=1.0f;
//	    out_limit[FR] = power_limit_k * out_incr[FR];
//	    out_limit[FL] = power_limit_k * out_incr[FL];
//	    out_limit[BL] = power_limit_k * out_incr[BL];
//	    out_limit[BR] = power_limit_k * out_incr[BR];	
//	}
//}

//void power_no_limit()//功率控制函数
//{
//   out_limit[FR] = out_incr[FR];
//	 out_limit[FL] = out_incr[FL];
//	 out_limit[BL] = out_incr[BL];
//	 out_limit[BR] = out_incr[BR];
//}
void K1_K2_INIT(void)
{
	  K1_3508[0]=250.975f;
	  K2_3508[0]=0.00000018f;
	
	  K1_3508[1]=250.975f;
	  K2_3508[1]=0.00000018f;
	
	  K1_3508[0]=250.975f;
	  K2_3508[0]=0.00000018f;
	
	  K1_3508[0]=250.975f;
	  K2_3508[0]=0.00000018f;
	
	  K1_6020[0]=11.5f;
	  K2_6020[0]=0.000005464f;
	
		K1_6020[1]=11.5f;
	  K2_6020[1]=0.000005464f;
	
		K1_6020[2]=11.5f;
	  K2_6020[2]=0.000005464f;
		
		K1_6020[3]=11.5f;
	  K2_6020[3]=0.000005464f;

}

float DATA_T1(float w,float P,double k1,double k2)
{
   float t1;
		
	 float mid_temp;
	 mid_temp=pow((w/9.55f), 2)-4.0f*k1*(k2*w*w-P);
	 if(mid_temp<=0)
	 {
	   return 0;
	 }
	 t1=((-w/9.55f)+sqrt(mid_temp))/(2.0f*k1);
	 return t1;

}
float DATA_T2(float w,float P,double k1 ,double k2)
{
   float t1;
		
	 float mid_temp;
	 mid_temp=pow((w/9.55f), 2)-4.0f*k1*(k2*w*w-P);
	 if(mid_temp<=0)
	 {
	   return 0;
	 }
	 t1=((-w/9.55f)-sqrt(mid_temp))/(2.0f*k1);
	 return t1;


}

//通过PID的P的值判断当前选用那个结果
float return_real_cmd(float t1,float t2,float PID_P,float v)
{	  
//     //这里默认t1>t2  不好用
//	  if(PID_P>=0)
//		{
//      if(v>=0)//电机正转而且目标值大于实际值为加速
//		  {
//		    return t1;
//		  }
//			else
//			{
//			  return t1;
//			}
//	  }
//		else
//		{
//	  	if(v<0)//电机正在反转而且目标值反向大于实际值，反向加速
//		  {
//        return t2;		   
//		  }
//			else
//			{
//			  return t2;
//			}
//	  }

    if(PID_P>=0)
		{     
			  return t1;
	  }
		else
		{
	  	  return t2;
		}

}
float MP1,MP2,MP3,MP4,MP5,MP6,MP7,MP8;
float velo1, velo2, velo3, velo4, velo5, velo6, velo7, velo8;
float cmd1, cmd2, cmd3, cmd4, cmd5, cmd6, cmd7, cmd8;
float P_ALL; 
float power_limit_k;

extern int16_t piderror[4];

void power_limit_duo_lun(float pow_max,int16_t *M_i_3508,int16_t*M_i_6020)
{
  	velo1 = ((float)WHEEL_3508_I_MOTOR[0].re_speed) ;
  	velo2 = ((float)WHEEL_3508_I_MOTOR[1].re_speed) ;
  	velo3 = ((float)WHEEL_3508_I_MOTOR[2].re_speed) ;
  	velo4 = ((float)WHEEL_3508_I_MOTOR[3].re_speed) ;

		cmd1 = ((float)M_i_3508[0])  *T_CMD_3508;
		cmd2 = ((float)M_i_3508[1])  *T_CMD_3508;
		cmd3 = ((float)M_i_3508[2])  *T_CMD_3508;
		cmd4 = ((float)M_i_3508[3])  *T_CMD_3508;

		MP1=K1_3508[0]*cmd1*cmd1+K2_3508[0]*velo1*velo1+cmd1*velo1/9.55f;
		MP2=K1_3508[1]*cmd2*cmd2+K2_3508[1]*velo2*velo2+cmd2*velo2/9.55f;
		MP3=K1_3508[2]*cmd3*cmd3+K2_3508[2]*velo3*velo3+cmd3*velo3/9.55f;
		MP4=K1_3508[3]*cmd4*cmd4+K2_3508[3]*velo4*velo4+cmd4*velo4/9.55f;

  	P_ALL=MP1+MP2+MP3+MP4;
  	power_limit_k=pow_max/P_ALL;

    float t1,t2;
		
		if(power_limit_k<=1&&power_limit_k>=0)
		{
			 MP1=power_limit_k*MP1;
			 MP2=power_limit_k*MP2;
			 MP3=power_limit_k*MP3;
			 MP4=power_limit_k*MP4;

			 if(MP1<=0)//单个电机负功不缩放
			 {
					out_3508[0] =M_i_3508[0];
			 }	
			 else
			 {
					t1=DATA_T1(velo1,MP1,K1_3508[0],K2_3508[0]);
					t2=DATA_T2(velo1,MP1,K1_3508[0],K2_3508[0]);
					out_3508[0]=return_real_cmd(t1,t2,piderror[0],velo1)/(T_CMD_3508);
			 }		

			 if(MP2<=0)//单个电机负功不缩放
			 {
					out_3508[1] =M_i_3508[1];
			 }	
			 else
			 {
					t1=DATA_T1(velo2,MP2,K1_3508[1],K2_3508[1]);
					t2=DATA_T2(velo2,MP2,K1_3508[1],K2_3508[1]);
					out_3508[1]=return_real_cmd(t1,t2,piderror[1],velo2)/(T_CMD_3508);
			 }

			 if(MP3<=0)//单个电机负功不缩放
			 {
					out_3508[2] =M_i_3508[2];
			 }	
			 else
			 {
					t1=DATA_T1(velo3,MP3,K1_3508[2],K2_3508[2]);
					t2=DATA_T2(velo3,MP3,K1_3508[2],K2_3508[2]);
					out_3508[2]=return_real_cmd(t1,t2,piderror[2],velo3)/(T_CMD_3508);
			 }

			 if(MP4<=0)//单个电机负功不缩放
			 {
					out_3508[3] =M_i_3508[3];
			 }	
			 else
			 {
					t1=DATA_T1(velo4,MP4,K1_3508[3],K2_3508[3]);
					t2=DATA_T2(velo4,MP4,K1_3508[3],K2_3508[3]);
					out_3508[3]=return_real_cmd(t1,t2,piderror[3],velo4)/(T_CMD_3508);
			 }	
		}	
		else
		{	//做负功或者没到Pmax，不用缩放
				power_limit_k=1.0;
				out_3508[0] = power_limit_k * M_i_3508[0];
				out_3508[1] = power_limit_k * M_i_3508[1];
				out_3508[2] = power_limit_k * M_i_3508[2];
				out_3508[3] = power_limit_k * M_i_3508[3];	
			
//				out_6020[0] = power_limit_k * M_i_6020[0];
//				out_6020[1] = power_limit_k * M_i_6020[1];
//				out_6020[2] = power_limit_k * M_i_6020[2];
//				out_6020[3] = power_limit_k * M_i_6020[3];	
		}
//		    out_6020[0] = 1 * M_i_6020[0];
//				out_6020[1] = 1 * M_i_6020[1];
//				out_6020[2] = 1 * M_i_6020[2];
//				out_6020[3] = 1 * M_i_6020[3];	
}

//功率控制

//3508 C620电机电流发送值为+16384____ -16384  对应电流为+20A__-20A
//于是有   力矩电流=20/16384*电流原始值
//然后求出力矩  力矩=Kt*力矩电流
//根据3508手册+电机减速比可以推算  Kt= 0.3*187/3591=0.01562N.M/A

//无刷电机输入功率=机械功率+铜损+磁损+控制器静态功率 
//机械功率占比较高   机械功率=力矩*转速/9.55
//9.55为转化系数，推导暂无

//电机功率模型（经验公式）   输入功率=机械功率+K1*转速*转速+K2*力矩*力矩+a(静态功耗)
//输入功率为Pmax  ，力矩为t，转速为w则有

//Pmax=K1*t*t+K2*w*w+w*t/9.55 

//在最大功率，转速，已知情况下  求解力矩t

//t1=-w/9.55+（根号（w/9.55平方））/2*k1、
//t2=   ----------------
//一般会存在俩个解，一个是限功率加速力矩，一个是限制功率减速力矩
//需要根据原始PID输出电流方向进行匹配

//进行整车功率限制时，使用功率再分配

// k=总体限制功率/设定功率
// 每个电机的实际输出功率=k*单个电机的设定功率
//如果功率输出方向为负，可不用限制功率
//然后根据每个电机分配的功率反推出转矩，进而推算出力矩电流



