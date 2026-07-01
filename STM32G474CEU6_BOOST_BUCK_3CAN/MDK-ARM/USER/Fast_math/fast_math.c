#include "fast_math.h"
#include "math.h"
/* Define PI */ 
#define PI 3.1415926536f 
/* Define Q31 */ 
#define Q31 0x80000000 
/* Define Q31 to float unit in RADIAN = Q31/PI */
#define RADIAN_Q31_f 683565275.6f   //弧度转换Q31定点量
#define RADIAN_Q31_j 11930464.71f   //角度转换Q31定点量


float fast_abs(float in)
{
	return(in >= 0 ? in : -in);
}



//快速开根号
float InvSqrt(float x)
{
 if(x==0){return 0;}
 float xhalf = 0.5f*x;
 int32_t i = *(int32_t*)&x;        // get bits for floating VALUE
 i = 0x5f375a86- (i>>1);           // gives initial guess y0
 x = *(float*)&i;                  // convert bits BACK to float
 x = x*(1.5f-xhalf*x*x);           // Newton step, repeating increases accuracy
 x = 1 / x;
 return x;
}


float FindList_COS(float in)
{
   return 0;
}

CCMRAM void Calculate_fix_sin_cos(int32_t angle,int32_t *sin,int32_t *cos)
{
	 //配置cst寄存器计算sin/cos
   CORDIC->CSR=0X00180061;  
	 //向wdata寄存器写入角度值
   CORDIC->WDATA=angle;
   
	//获得sin/cos
	*sin=CORDIC->RDATA;
	*cos=CORDIC->RDATA;

}
//输入角度范围[-180, 180] 
//输入弧度数据 弧度=角度*pi/180
CCMRAM void Calculate_Float_Sin_Cos(float angle,float *sin, float *cos) 
{ 
 /* Q31,two write, two read, sine calculate, 6 precision */ 
 CORDIC->CSR = 0x00180061; 
	
 while(angle>180)
 {
	 angle-=360;
 }
 
 while(angle<-180)
 {
	 angle+=360;
 }
  
 /* Write data into WDATA */ 
 CORDIC->WDATA = (int32_t )(angle * RADIAN_Q31_j); 
  
 /* Modulus is m=1 */ 
 CORDIC->WDATA = 0x7FFFFFFF; 
  
 /* Get sin value in float */ 
 *sin = ((int32_t)CORDIC->RDATA)*1.0f/Q31; 
 
 /* Get cos value in float */ 
 *cos = ((int32_t)CORDIC->RDATA)*1.0f/Q31;     
} 

CCMRAM void Calculate_Float_Sin(float angle,float *sin) 
{ 
 /* Q31,two write, one read, sine calculate, 6 precision */ 
 CORDIC->CSR = 0x00100061; 
  
 /* Write data into WDATA */ 
 CORDIC->WDATA = (int32_t )(angle * RADIAN_Q31_j); 
  
 /* Modulus is m=1 */ 
 CORDIC->WDATA = 0x7FFFFFFF; 
  
 /* Get sin value in float */ 
 *sin = ((int32_t)CORDIC->RDATA)*1.0f/Q31;	
} 





//input [-1,+1]   outangle [-45,+45]
CCMRAM float Calculate_Float_arctan(float data) 
{ 
 float angle;
 /* Q31,two write, one read, sine calculate, 6 precision */ 
 CORDIC->CSR = 0x00000064; 
  
 /* Write data into WDATA */ 
 CORDIC->WDATA = (int32_t )(data * Q31);  
  
 /* Get sin value in float */ 
 angle = ((int32_t)CORDIC->RDATA)*1.0f/RADIAN_Q31_j;	
	 
 return angle;
} 
//快速tan
CCMRAM void Calculate_Float_arctan_all(float x,float y,float *angle)
{
   uint8_t state=(((uint8_t)(x>0))<<2)+(((uint8_t)(y>0))<<1)+((_fabsf(x))>_fabsf(y));
   if(x==0&&y>0)     {*angle=90.0f;return;}
	 else if(x==0&&y<0){*angle=-90.0f;return;}
	 
	 if(x>0&&y==0)     {*angle=0.0f;return;}
	 else if(x<0&&y==0){*angle=180.0f;return;}
	 
	 if(x==0&&y==0)    {*angle=0.0f;return;}
	
	 switch (state)
	 {
		 case 0x00:  *angle=-90.0f-Calculate_Float_arctan(x/y)  ;break;
		 case 0x01:  *angle=-180.0f+Calculate_Float_arctan(y/x) ;break;
		 case 0x02:  *angle=90.0f+Calculate_Float_arctan(-x/y)  ;break;
	   case 0x03:  *angle=180.0f-Calculate_Float_arctan(-y/x) ;break;
		 case 0x04:  *angle=-90.0f+Calculate_Float_arctan(-x/y) ;break;
	   case 0x05:  *angle=-Calculate_Float_arctan(-y/x)       ;break;
	   case 0x06:  *angle=90.0f-Calculate_Float_arctan(x/y)   ;break;
	   case 0x07:  *angle=Calculate_Float_arctan(y/x)         ;break;
		 default: break;
	 }
	 return;
}

/**
  * @brief    LPF_Init
  * @note    LOW_PASS_FILTER Initialize
  * @param  	LPF 低通滤波器的指针
							threshold	低通滤波器的阈值
  * @retval 	none
  * @author  RobotPilots
  */
void LPF_Init(LOW_Pass_Filter *LPF, float threshold)
{
	LPF->last = 0;
	LPF->now = 0;
	LPF->threshold = threshold;
	LPF->output = 0;
	LPF->High_flag = 0;
}

/**
  * @brief    LPF_Init
  * @note    LOW_PASS_FILTER Initialize
  * @param  	LPF 低通滤波器的指针
							input_data 进入低通滤波的数据
  * @retval 	none
  * @author  RobotPilots
  */
float LPF_add(LOW_Pass_Filter *LPF, float input_data)
{
  LPF->now=input_data;
	
	LPF->output = LPF->now*LPF->threshold+LPF->last*(1-LPF->threshold);
	
	LPF->last=LPF->now;
	
	return LPF->output;
}







