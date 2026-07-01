#include "u_i_pid.h"
#include "math.h"

CCMRAM PID CAP_charge_I; //电容充电电流环  //输入设定电流，调节实际pwm
CCMRAM PID CAP_charge_V; //电容充电电压环  //输入设定电压，调节实际pwm
CCMRAM PID CAP_charge_P; //电容充电功率环  //输入设定功率，调节实际功率
CCMRAM PID CAP_buffer_P; //电容缓冲能量环  //输入设定功率，调节实际功率

CCMRAM PID PID_3508_speed_motor[4];
CCMRAM PID PID_6020_speed_motor[4];
CCMRAM PID PID_6020_angle_motor[4];


void pid_init(PID *pid,float kp,float ki,float kd, float omax,float omin, float pmax, float imax, float dmax)
{
	pid->OMAX=omax;
	pid->PMAX=pmax;
	pid->IMAX=imax;
	pid->DMAX=dmax;
	pid->OMIN=omin;
	
	pid->Kp=kp;
	pid->Ki=ki;
	pid->Kd=kd;
	
	pid->out=0.0f;
	pid->Pout=0.0f;
	pid->Iout=0.0f;
	pid->Dout=0.0f; 
	pid->error[0]=0.0f;
	pid->error[1]=0.0f;
	pid->error[2]=0.0f;
	pid->set=0.0f;
	pid->real=0.0f;
	pid->sum_error=0.0f;
}
CCMRAM float  PID_Position(PID *Pid,float real,float set)
{ Pid->set = set;
	Pid->real = real;
	Pid->error[0] = set - real;
	
	if(fabs(Pid->error[0])<Pid->interget_seperate){
	Pid->sum_error += Pid->error[0];}
    
	if(Pid->sum_error>Pid->sum_error_max) {Pid->sum_error=Pid->sum_error_max;}
	if(Pid->sum_error<-Pid->sum_error_max) {Pid->sum_error=-Pid->sum_error_max;}

	Pid->Pout=Pid->Kp * Pid->error[0] ;
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;

	Pid->Iout=Pid->Ki * Pid->sum_error ;
	Pid->Iout > Pid->IMAX ? Pid->Iout=Pid->IMAX :1;
	Pid->Iout < -Pid->IMAX ?Pid->Iout = -Pid->IMAX :1;

	Pid->Dout=Pid->Kd * ( Pid->error[0] - Pid->error[1] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;
	
	Pid->out=Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OMAX ? Pid->out = Pid->OMAX : 1;
	Pid->out < Pid->OMIN ? Pid->out = Pid->OMIN : 1;

	Pid->error[1] = Pid->error[0];	

	return Pid->out;

}    

CCMRAM float  PID_Position_DPYAW(PID *Pid,float real,float set)
{ Pid->set = set;
	Pid->real = real;
	Pid->error[0] = set - real;
	
	if(fabs(Pid->error[0])<15){
	Pid->sum_error += Pid->error[0];}
    
	if(Pid->sum_error> Pid->sum_error_max) {Pid->sum_error= Pid->sum_error_max;}
	if(Pid->sum_error<-Pid->sum_error_max) {Pid->sum_error=-Pid->sum_error_max;}
	
	float temp_kp;
	
	temp_kp=190*(50/(fabs(Pid->error[0])));
	temp_kp >300 ? temp_kp = 300 : 1;
	temp_kp < 150 ? temp_kp = 150 : 1;	

	Pid->Pout=temp_kp* Pid->error[0] ;
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;

	Pid->Iout=Pid->Ki * Pid->sum_error ;
	Pid->Iout > Pid->IMAX ? Pid->Iout=Pid->IMAX :1;
	Pid->Iout < -Pid->IMAX ?Pid->Iout = -Pid->IMAX :1;

	Pid->Dout=Pid->Kd * ( Pid->error[0] - Pid->error[1] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;
	
	Pid->out=Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OutMAX ? Pid->out = Pid->OutMAX : 1;
	Pid->out < -Pid->OutMAX ? Pid->out = -Pid->OutMAX : 1;

	Pid->error[1] = Pid->error[0];	

	return Pid->out;

}

CCMRAM float  PID_Position_DP(PID *Pid,float real,float set)
{ Pid->set = set;
	Pid->real = real;
	Pid->error[0] = set - real;
	
	if(fabs(Pid->error[0])<Pid->interget_seperate){
	Pid->sum_error += Pid->error[0];}
    
	if(Pid->sum_error>Pid->sum_error_max) {Pid->sum_error=Pid->sum_error_max;}
	if(Pid->sum_error<-Pid->sum_error_max) {Pid->sum_error=-Pid->sum_error_max;}

	Pid->Pout=Pid->Kp * Pid->error[0] ;
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;

	Pid->Iout=Pid->Ki * Pid->sum_error ;
	Pid->Iout > Pid->IMAX ? Pid->Iout=Pid->IMAX :1;
	Pid->Iout < -Pid->IMAX ?Pid->Iout = -Pid->IMAX :1;

	Pid->Dout=Pid->Kd * ( Pid->error[0] - Pid->error[1] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;
	
	Pid->out=Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OutMAX ? Pid->out = Pid->OutMAX : 1;
	Pid->out < -Pid->OutMAX ? Pid->out = -Pid->OutMAX : 1;

	Pid->error[1] = Pid->error[0];	

	return Pid->out;

}

void PID_IS_Init(PID* Pid,float kp,float ki,float kd,float pmax,float imax,float dmax,float outmax,float sum_error_max,float interget_seperate)
{

	Pid-> set=0;
	Pid-> real=0;
	Pid-> Pout=0;
	Pid-> Iout=0;
	Pid-> Dout=0;
	Pid-> out=0;
	
	Pid-> error[0]=0.0f;
	Pid-> error[1]=0.0f;
	Pid-> error[2]=0.0f;
	Pid->sum_error=0;
	Pid->interget_seperate=interget_seperate;
	Pid->sum_error_max=sum_error_max;
	
	Pid-> Kp=kp;
	Pid-> Ki=ki;
	Pid-> Kd=kd;
	
	Pid-> PMAX=pmax;
	Pid-> IMAX=imax;
	Pid-> DMAX=dmax;
	Pid->OutMAX=outmax;
}
//6020舵轮角度专用解算pid
CCMRAM float MAKE_ANGLE_TO_0_360(float angle)
{
	float in_data=angle;
	 if(in_data<0)
	 {
	    while(in_data<0)
			{
			   in_data+=360;
			}
	 }
	 if(in_data>=360)
	 {
	   while(in_data>=360)
	   {  
	      in_data-=360;
	   }
	 }
   return in_data;
}


CCMRAM float  PID_Position_6020(PID *Pid,float real,float set,int16_t*M3508V_OFFSET)
{//位置
  //if set 0 real 10  error -10
	//if ser 0 real 350 error  -350 +10
	//if set 100 real 120  error -20
	//if set 100 real 290  error -190 +150
	//if ser 350 real 10  340    -20

	//当3508速度偏移负方向时候，电机控制角度偏移180
  
	
	Pid->set =  MAKE_ANGLE_TO_0_360(set);
	Pid->real = MAKE_ANGLE_TO_0_360(real);
	Pid->error[0] = set - real;
	if(Pid->error[0]>180){Pid->error[0]-=360;}
  if(Pid->error[0]<-180){Pid->error[0]+=360;}
	
	
	//进行完0-360映射后，实际控制角度范围可以优化
  //控制误差大于90可以使轮子反转反向接近实际输出角度	
	if(Pid->error[0]>90)
	{
	  *M3508V_OFFSET=-1;
	  //切换3508工作模式
	  Pid->error[0]=Pid->error[0]-180;
	}
	else if(Pid->error[0]<-90)
	{
	  *M3508V_OFFSET=-1;
	  //切换3508工作模式
	  Pid->error[0]=180+Pid->error[0];
	}
	else
	{
	  *M3508V_OFFSET=1;
	}
		
	
	Pid->sum_error += Pid->error[0];

	Pid->Pout=Pid->Kp * Pid->error[0];
	
  
	
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;

	Pid->Iout=Pid->Ki * Pid->sum_error ;
	
	Pid->Iout > Pid->IMAX ? Pid->Iout=Pid->IMAX :1;
	Pid->Iout < -Pid->IMAX ?Pid->Iout = -Pid->IMAX :1;

	Pid->Dout=Pid->Kd * ( Pid->error[0] - Pid->error[1] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;
	
	Pid->out=Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OMAX ? Pid->out = Pid->OMAX : 1;
	Pid->out < -Pid->OMAX ? Pid->out = -Pid->OMAX : 1;

	Pid->error[1] = Pid->error[0];
	return Pid->out;

}


//速度环专用PID
CCMRAM float  PID_Position_V(PID *Pid,float real,float set,float move_ment)
{//位置

	Pid->set = set;
	Pid->real = real;
	Pid->error[0] = set - real;
	
	Pid->sum_error += Pid->error[0];
	Pid->sum_error=Pid->sum_error-move_ment;

	Pid->Pout = Pid->Kp * Pid->error[0] ;
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;

	Pid->Iout=Pid->Ki * Pid->sum_error ;
	
	Pid->sum_error > Pid->IMAX ? Pid->sum_error=Pid->IMAX :1;
	Pid->sum_error < -Pid->IMAX ?Pid->sum_error = -Pid->IMAX :1;

	Pid->Dout=Pid->Kd * ( Pid->error[0] - Pid->error[1] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;
	
	Pid->out=Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OMAX ? Pid->out = Pid->OMAX : 1;
	Pid->out < -Pid->OMAX ? Pid->out = -Pid->OMAX : 1;

	Pid->error[1] = Pid->error[0];
	

	return Pid->out;
	
	
}


//增量环
CCMRAM float  PID_Incremental(PID *Pid,float real,float set)
	{//增量
	Pid->set = set;
	Pid->real = real;
	Pid->error[0] =   Pid->set - Pid->real;

	Pid->Pout = Pid->Kp * Pid->error[0];
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;
	
	Pid->Iout =	Pid->Ki * Pid->error[0];
	Pid->Iout > Pid->IMAX ? Pid->Iout = Pid->IMAX : 1;
	Pid->Iout < -Pid->IMAX ? Pid->Iout = -Pid->IMAX : 1;

	Pid->Dout =	Pid->Kd * ( Pid->error[0] - 2 * Pid->error[1] + Pid->error[2] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;

	Pid->error[2] = Pid->error[1];
	Pid->error[1] = Pid->error[0];
	
	Pid->out += (Pid->Pout + Pid->Iout + Pid->Dout);
	Pid->out > Pid->OMAX ? Pid->out = Pid->OMAX : 1;
	Pid->out < 0 ? Pid->out = 0 : 1;	
	return Pid->out;
}


//角度环专用PID
CCMRAM float PID_Position_ANGLE(PID *Pid,float real,float set)
{//位置

	Pid->set = set;
	Pid->real = real;
	Pid->error[0] = set - real;
	if((Pid->error[0])>180)
	{
	     Pid->error[0]-=360;
	}
	if(Pid->error[0]<-180)
	{
	    Pid->error[0]+=360;
	}
	
	Pid->sum_error += Pid->error[0];

	Pid->Pout=Pid->Kp * Pid->error[0];
	

	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;

	Pid->Iout=Pid->Ki * Pid->sum_error ;

	
	Pid->sum_error > Pid->IMAX ? Pid->sum_error=Pid->IMAX :1;
	Pid->sum_error < -Pid->IMAX ?Pid->sum_error = -Pid->IMAX :1;

	Pid->Dout=Pid->Kd * ( Pid->error[0] - Pid->error[1] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;
	
	Pid->out=Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OMAX ? Pid->out = Pid->OMAX : 1;
	Pid->out < -Pid->OMAX ? Pid->out = -Pid->OMAX : 1;

	Pid->error[1] = Pid->error[0];
	

	return Pid->out;
	
	
}


//只有p的增量环用于计算
float  PID_Incremental_lite(PID *Pid,float real,float set)
	{//增量
	Pid->set = set;
	Pid->real = real;
	Pid->error[0] =   Pid->set - Pid->real;

	Pid->Pout = Pid->Kp * (Pid->error[0]-Pid->error[1]);
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;
	
	Pid->Iout =	Pid->Ki * Pid->error[0];
	Pid->Iout > Pid->IMAX ? Pid->Iout = Pid->IMAX : 1;
	Pid->Iout < -Pid->IMAX ? Pid->Iout = -Pid->IMAX : 1;

	Pid->Dout =	Pid->Kd * ( Pid->error[0] - 2 * Pid->error[1] + Pid->error[2] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;

	Pid->error[2] = Pid->error[1];
	Pid->error[1] = Pid->error[0];
	
	Pid->out += Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OMAX ? Pid->out = Pid->OMAX : 1;
	Pid->out < Pid->OMIN ? Pid->out = Pid->OMIN : 1;
	
	return Pid->out;	
}
	





