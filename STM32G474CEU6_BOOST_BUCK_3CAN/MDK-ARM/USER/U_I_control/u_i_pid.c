#include "u_i_pid.h"

extern RC rc;

#define PWM  27200                   //200khz
#define Find_abs(x) 					       ((x)>0? (x):(-(x)))
#define Find_max(a,b)                ((a)>(b) ? (a):(b))
#define Find_min(a,b)                ((a)<(b) ? (a):(b))
//#include "key_encoder.h"
//电容电压上下限 缓冲能量上限阈值
#define CAP_CHARG_V_MAX 30550
#define CAP_CHARG_V_MIN 4000
#define BUFFER_ENERGY_MAX 60.0f
//pwm控制区间10%-90%
//HRTIM A电源端 //值越大电压越高 数值对应高端占空比
//HRTIM D电容端 //值越大电压越高 数值对应低端占空比
CCMRAM filter FLT_U_CAP;
CCMRAM filter FLT_I_CAP;
CCMRAM filter FLT_U_POW;
CCMRAM filter FLT_I_POW;//电压电流低通滤波器变量

uint32_t pow_ducy;
uint32_t cap_ducy;

CCMRAM float POWER_MAX=28;   //输入功率最大值
CCMRAM float set_I;
CCMRAM float set_U;
CCMRAM float set_P;
CCMRAM float buffer_P;

CCMRAM uint16_t RUN_TIME_TEST;


CCMRAM uint8_t protection_mode;
CCMRAM uint16_t time_boost;   //电压环计数器




//float angle=30;
//float sin_x=0;
//float cos_x=0;
//float ANGLE=10;
//float X=0.5;
//float Y=0.5;
//float inputx_y;
//uint32_t aaa;

uint8_t time6020ctrl_set;
uint8_t time_set;
uint16_t count_set;

CCMRAM void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	 if(htim->Instance == htim6.Instance)//1khz
	 {  
		 time6020ctrl_set++;
		 if(time6020ctrl_set%2==0)
		 {
		   //Motor_DUOLUN_PID_CTRL();
			 //Single_motor_power_test();
			 CHASSIS_CTRL(); 
		 }
		 //按键扫描
		 KEY_scan();
		  //__HAL_TIM_SET_COUNTER(&htim8,0);
		  //HAL_TIM_Base_Start(&htim8);
		 
			//U_I_CTRL_10000HZ();
		 
		  //RUN_TIME_TEST = __HAL_TIM_GET_COUNTER(&htim8);
	    //aaa=htim8.Instance->CNT;
	 }
	 else if(htim->Instance == htim7.Instance)//100hz
	 {
		  //电容CAN通信
		  CAP_DATA_TX_IN_CAN(); 
		  //裁判系统数据解算
		  queue_out_umpire_system(&USART3_RX_queue);
		  //缓冲能量环调节
		  BUFFER_ENERGY_CTRL();
			//电容容量百分比转换
			CAPTURE_CAPCITY_SET();
		  //裁判系统UI绘制
		  run_demo();
		 
		 static int timelivectrl_set;
		 timelivectrl_set++;
		 if(timelivectrl_set%3==0)
		 {		 
		  rc.live_over=0;
		 }
		  
		 
//		 	__HAL_TIM_SET_COUNTER(&htim8,0);
//		  HAL_TIM_Base_Start(&htim8);
//		 
//		  Calculate_Float_Sin_Cos(angle,&sin_x,&cos_x);
//		  
//		  Calculate_Float_arctan_all(X,Y,&ANGLE);
//		  inputx_y=Y/X;
//		 
//		  aaa=htim8.Instance->CNT;
		 
		  //can通信速率统计
		  time_set++;
		  if(time_set>=100)
		  {
		    count_set=can_rx_data_num;
			  can_rx_data_num=0;
		    time_set=0;
		  }
		  //这里放点时序不重要的
	 }
}




//arg 低通滤波参数
CCMRAM float FLT_COM(filter *FLT,float in,float arg)
{
   FLT->Fdata[1]=in;
	 FLT->Fdata[2]=FLT->Fdata[1]*(1-arg)+FLT->Fdata[0]*arg;
   FLT->Fdata[0]=FLT->Fdata[1];
	 return  FLT->Fdata[2];
}


CCMRAM float pid_pwm_out;
CCMRAM float CC_MODE_PWM;
CCMRAM float CV_MODE_PWM;
//电压电流并环
CCMRAM void CAP_U_I_CTRL(void)
{
	  
//	   //电容电压保护
     set_U=CAP_CHARG_V_MAX;
	   //
//	   if(CAP_U_ADC_DATA.lowflt_out<=CAP_CHARG_V_MIN)
//		 {		 
//		   //if(set_I<0){set_I=0;CAP_charge_P.Pout=0;}		 
//		 }
		 //电容电压保护(深大(好使))
		 if(CAP_U_ADC_DATA.lowflt_out < (CAP_CHARG_V_MIN + 2.0f) && CAP_charge_P.out < 0)
     {
        CAP_charge_P.out *= Find_max(0.5f*(CAP_U_ADC_DATA.lowflt_out - CAP_CHARG_V_MIN), 0);
     }
//     if(CAP_U_ADC_DATA.lowflt_out > (CAP_CHARG_V_MAX - 2.0f) && CAP_charge_P.out > 0)
//     {
//        CAP_charge_P.out *= Find_max(0.5f*(CAP_CHARG_V_MAX - CAP_U_ADC_DATA.lowflt_out), 0);
//     }
		 set_I=CAP_charge_P.out;
		 //set_I=5000;
		 //set_U=9000;
     CC_MODE_PWM=PID_Incremental_lite(&CAP_charge_I,CAP_I_ADC_DATA.lowflt_out,set_I);
	   CV_MODE_PWM=PID_Incremental_lite(&CAP_charge_V,CAP_U_ADC_DATA.lowflt_out,set_U);
		 
		 //电压电流环竞争(大理)
	   if(CC_MODE_PWM<CV_MODE_PWM)
		 {
		   pid_pwm_out=CC_MODE_PWM;
			 //CAP_charge_V.out=CC_MODE_PWM;
		 }
		 else
		 {
		   pid_pwm_out=CV_MODE_PWM;
			 //CAP_charge_I.out=CV_MODE_PWM;
			 if(CAP_U_ADC_DATA.lowflt_out > (CAP_CHARG_V_MAX - 2.0f) && CAP_charge_P.out > 1000)
       {
				  CAP_charge_P.out = Find_max(0.5f*(CAP_CHARG_V_MAX - CAP_U_ADC_DATA.lowflt_out), 0)+1000;//输出限幅;
       }
		 }
}

CCMRAM void CAP_P_CTRL(void)
{
   if(HRTIM_BIT==ON)//hrtim开启，功率环开始工作
	 {
	   //set_P=10;
		 //set_I=set_P*1000.0f*1000.0f/CAP_U_ADC_DATA.lowflt_out;
     set_I=PID_Incremental_lite(&CAP_charge_P,POW_P,set_P);
   }
	 else
	 {
	    CAP_charge_P.out=0; //不能让输出提前增加
	 }
}

CCMRAM void CAP_P_SET(void)
{
	 //SET_P大于0电容充电，小于0电容放电
   if(HRTIM_BIT==ON)//有输入时才能控功
	 {	 
		 set_P=robot.chassis_power_limit-buffer_P;
		 //set_P=30-buffer_P; //设定裁判系统输出功率	 
		 set_P > 150 ? set_P = 150 : 1;
	   set_P < 35  ? set_P = 35 : 1;
   }
	 else
	 {
	   set_P=0; //不能让输出提前增加
	 }
}
CCMRAM void BUFFER_ENERGY_CTRL(void)
{
   if(HRTIM_BIT==ON)//有输入时才能控功
	 {
		 //buffer_P=PID_Incremental_lite(&CAP_buffer_P,CHASSIS_ENG,BUFFER_ENERGY_MAX/2);
	   buffer_P=PID_Position(&CAP_buffer_P,robot.buffer_energy,BUFFER_ENERGY_MAX*0.85f);
	 }
	 else
	 {
	    buffer_P=0;
	 }
}

CCMRAM void PID_CTRL()
{
//此处用于调试占空比	
	//pid_pwm_out占空比调节范围
	//0.10-0.90  降压
	//0.90-1.3   升压
	//pid_pwm_out=0.23;

	if(pid_pwm_out<=0.10f)
	{
	  pow_ducy=0.10f*PWM;
	  cap_ducy= PWM*0.10F;
	}
	else if(pid_pwm_out<=0.90f)
	{
	  pow_ducy=pid_pwm_out*PWM;
	  cap_ducy= PWM*0.10f;
	}
	else if(pid_pwm_out<=1.5f)
	{
	  cap_ducy=(pid_pwm_out-0.80f)*PWM;
	  pow_ducy= PWM*0.90f;	
	}
	else 
	{
	  cap_ducy=(1.5f-0.80f)*PWM;
	  pow_ducy= PWM*0.90f;
	}
	//MAX 57600
  
	HRTIM1->sMasterRegs.MCMP1R = (HRTIM1->sMasterRegs.MPER - pow_ducy) / 2u;
  HRTIM1->sMasterRegs.MCMP2R = (HRTIM1->sMasterRegs.MPER + pow_ducy) / 2u;
  HRTIM1->sMasterRegs.MCMP3R = (cap_ducy)/2u;
  HRTIM1->sMasterRegs.MCMP4R = (2*HRTIM1->sMasterRegs.MPER - cap_ducy)/ 2u;
}

//设定功率电容充电函数
void CAP_POWER_IN()
{
//   //cap_time_set++;
//   if(cap_time_set==10)
//	 { 
//	    cap_time_set=0; 
//	 }
}
CCMRAM void U_I_CTRL_10000HZ(void)
{
	 //数据采集		 
	 POWER_ANALYSE_ALL();
	 //安全检测
   //POWER_PROTECTION();	
	 POWER_PROTECTION_LITE();
	 //充放检测
	 CAP_P_SET();
	 CAP_P_CTRL();
	 CAP_U_I_CTRL();
	 PID_CTRL();
	 //pid运算
}

CCMRAM uint16_t cap_time_set=0; 
//电容充电功率环心跳计时器

//CCMRAM void CAP_U_I_CTRL_C(void)
//{
//	 //set_I=1000;
//	 //if(set_I>10000){set_I=10000;}
//	 //if(set_I<-10000){set_I=-10000;}
//	   //set_U=10000;
//		 set_I=PID_Incremental_lite(&CAP_charge_V,CAP_U_ADC_DATA.lowflt_out,set_U);
//	   //set_I=-3500;
//     CC_MODE_PWM=PID_Incremental_lite(&CAP_charge_I,CAP_I_ADC_DATA.lowflt_out,set_I);
//	   pid_pwm_out=CC_MODE_PWM;
//}
//单个电压环
//CCMRAM void CAP_I_CTRL(void)
//{
//	 if(HRTIM_BIT==ON)//hrtim开启，电流环开始工作
//	 {
//		 //恒定电流测试
//	   //set_I=1000;
//	   //if(set_I>10000){set_I=10000;}
//		 //if(set_I<-10000){set_I=-10000;}
//     pid_pwm_out=PID_Incremental_lite(&CAP_charge_I,CAP_I_ADC_DATA.lowflt_out,set_I);//100
//   }
//	 else
//	 {
//	    CAP_charge_I.out=0; //不能让输出提前增加
//	 }
//}
////单个电流环
//CCMRAM void CAP_U_CTRL(void)
//{ 
//	if(HRTIM_BIT==ON)//hrtim开启，电压环开始工作
//	 {
//		 //恒定电压测试
//	   //set_U=7000;
//	   //if(set_U>15000){set_U=15000;}
//		 //if(set_U<5000){set_U=5000;}
//     set_I=PID_Position(&CAP_charge_V,CAP_U_ADC_DATA.lowflt_out,set_U);//100
//   }
//	 else
//	 {
//	    CAP_charge_V.out=0; //不能让输出提前增加
//	 }
//	
//}
//CCMRAM void U_P_CTRL_1000HZ(void)
//{
//		 POWER_ANALYSE_1000HZ();
//		 //上电，等电压电流趋向稳定在控功
//		 if(cap_time_set>100)
//		 {
//		 CAP_P_SET();
//		 CAP_P_CTRL();
//		 }
//		 else
//		 {
//			 cap_time_set++;
//		 }
//		 CAP_U_CTRL ();
//}
// 参考深圳大学代码
//void pwm_output()
//{
//  //这里参考安合科技的程序
//  //控制思路在资料《基于STM32F334同步整流BUCK-BOOST数字电源设计》P8有详细说明
//  uint32_t	buck_duty,boost_duty;
//	uint32_t	PWM_PER_0_5	= 0.5f * hhrtim1.Instance->sTimerxRegs[0].PERxR;
//    
//	if(pid_cap_i.OutDutyCycle > 0.9f)  //当总占空比大于90%时，进入BOOST模式。
//		boost_duty = (pid_cap_i.OutDutyCycle - 0.8f) * PWM_PER_0_5;//设置boost的占空比，并且模拟中心对称的PWM。注意要减去buck_duty的占空比: 0.8*PWM_PER_0_5
//	else
//		boost_duty = 0.1f * PWM_PER_0_5;	//当总占空比不大于90%时，进入BUCK模式,boost_duty给固定占空比

//	if(pid_cap_i.OutDutyCycle > 0.9f)
//		buck_duty = 0.9f * PWM_PER_0_5; //当总占空比大于90%时，进入BOOST模式,buck_duty给固定占空比
//	else
//		buck_duty = pid_cap_i.OutDutyCycle * PWM_PER_0_5;	//设置buck的占空比，并且模拟中心对称的PWM。

//	  hhrtim1.Instance->sTimerxRegs[0].CMP1xR = PWM_PER_0_5 - buck_duty;//TimerA PWM置1比较器，模拟中心对称的PWM，以PWM_PER_0_5为中心
//    hhrtim1.Instance->sTimerxRegs[0].CMP2xR = PWM_PER_0_5 + buck_duty;//TimerA PWM置0比较器，比如  （400-50）与（400+50）的中心点为400	

//	  hhrtim1.Instance->sTimerxRegs[3].CMP1xR = PWM_PER_0_5 + boost_duty;//TimerD PWM置1比较器，模拟中心对称的PWM，以PWM_PER_0_5为中心
//    hhrtim1.Instance->sTimerxRegs[3].CMP2xR = PWM_PER_0_5 - boost_duty;//TimerD PWM置0比较器，比如  （400-50）与（400+50）的中心点为400	
//	
//}

//电容分boost 升压和buck 降压，当pwm 频率适合时，电压会正向流动，当pwm频率不足以给电容充电时，电流逆向流动


	//pow_ducy= Encoder*1000;;  //B       //越小越打开地
	//cap_ducy= PWM*0.9F;         //A       //越小越打开地 
	//extern uint16_t Encoder;

  //	if(pid_cap_i.OutDutyCycle > 0.9f)  //当总占空比大于90%时，进入BOOST模式。
  //		boost_duty = (pid_cap_i.OutDutyCycle - 0.8f) * PWM_PER_0_5;//设置boost的占空比，并且模拟中心对称的PWM。注意要减去buck_duty的占空比: 0.8*PWM_PER_0_5
  //	else
//		boost_duty = 0.1f * PWM_PER_0_5;	//当总占空比不大于90%时，进入BUCK模式,boost_duty给固定占空比

//	if(pid_cap_i.OutDutyCycle > 0.9f)
//		buck_duty = 0.9f * PWM_PER_0_5; //当总占空比大于90%时，进入BOOST模式,buck_duty给固定占空比
//	else
//		buck_duty = pid_cap_i.OutDutyCycle * PWM_PER_0_5;	//设置buck的占空比，并且模拟中心对称的PWM。
//  pid_pwm_out=0.5f;
	
//	if(pid_pwm_out>0.95f)   //当总体占空比大于90%，进入升压boost
//	{
//	   cap_ducy = (1.0f-(pid_pwm_out - 0.88f)) * PWM;
//	}
//	else   //buck
//	{
//	   cap_ducy= PWM*0.95F;  
//	}
//	
//	if(pid_pwm_out>0.95f)  //boost
//	{
//	   pow_ducy=0.95f * PWM;
//	}
//	else  //buck
//	{
//	   pow_ducy=pid_pwm_out*PWM;
//	}






