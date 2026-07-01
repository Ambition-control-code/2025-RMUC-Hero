#include "u_i_pid.h"

PID CAP_charge_I; //电容充电电流环  //输入设定电流，调节实际pwm
PID CAP_charge_V; //电容充电电压环  //输入设定电压，调节实际pwm
PID CAP_charge_P; //电容充电功率环  //输入设定功率，调节实际功率



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
//速度环  PID控制
float  PID_Position(PID *Pid,float real,float set)
{//位置

	Pid->set = set;
	Pid->real = real;
	Pid->error[0] = set - real;
	
	Pid->sum_error += Pid->error[0];

	Pid->Pout=Pid->Kp * Pid->error[0];
	
	if(Pid->Pout>0)
	{
	   Pid->Pout+=70;
	}
	if(Pid->Pout<0)
	{
	   Pid->Pout-=70;
	}
	
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
float  PID_Position_V(PID *Pid,float real,float set,float move_ment)
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
float  PID_Incremental(PID *Pid,float real,float set)
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
float PID_Position_ANGLE(PID *Pid,float real,float set)
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
	
	Pid->out += Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OMAX ? Pid->out = Pid->OMAX : 1;
	Pid->out < Pid->OMIN ? Pid->out = Pid->OMIN : 1;

	
	return Pid->out;
	
	
}
	





