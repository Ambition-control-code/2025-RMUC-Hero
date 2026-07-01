#include "u_i_pid.h"
//#include "key_encoder.h"

//死区间隙3.33% 即pwm控制区间3%-97%
//HRTIM A电源端 //值越大电压越高 数值对应高端占空比
//HRTIM D电容端 //值越大电压越高 数值对应低端占空比
CCMRAM filter FLT_U_CAP;
CCMRAM filter FLT_I_CAP;
CCMRAM filter FLT_U_POW;
CCMRAM filter FLT_I_POW;//电压电源低通滤波器变量

CCMRAM uint32_t pow_ducy;
CCMRAM uint32_t cap_ducy;

CCMRAM float POWER_MAX=28;   //输入功率最大值
CCMRAM float set_I;
CCMRAM float set_U;
CCMRAM float set_P;
CCMRAM float CTRL_P;

CCMRAM uint16_t RUN_TIME_TEST;


CCMRAM uint8_t protection_mode;
CCMRAM uint16_t time_boost;   //电压环计数器

CCMRAM void U_I_CTRL_10000HZ(void);

CCMRAM void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	 if(htim->Instance == htim6.Instance)//100us
	 {  
		  //__HAL_TIM_SET_COUNTER(&htim8,0);
		  //HAL_TIM_Base_Start(&htim8);
		 
			U_I_CTRL_10000HZ();
		 
		  //RUN_TIME_TEST = __HAL_TIM_GET_COUNTER(&htim8);
	    //aaa=htim6.Instance->CNT;
	 }
	 if(htim->Instance == htim7.Instance)
	 {
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

#define PWM  27200

CCMRAM float pid_pwm_out;
CCMRAM float CC_MODE_PWM;
CCMRAM float CV_MODE_PWM;

//电压电流并环
CCMRAM void CAP_U_I_CTRL(void)
{

	   //电容充电应该有电压范围限制
	   //电压最大值可由电压环与电流环并路输出实现,二者取小可以限制电压电流的最大值 
     //电容欠压时应该控功充电	
	   //set_I>0充电
			 set_U=CAP_MAX_V;
			 if(CAP_U_ADC_DATA.lowflt_out<CAP_MIN_V)//小于最小电压阶段控功充电
			 {
				 if(set_I<10){ set_I=10;}
			 }
       CC_MODE_PWM=PID_Incremental_lite(&CAP_charge_I,CAP_I_ADC_DATA.lowflt_out,set_I);
	     CV_MODE_PWM=PID_Incremental_lite(&CAP_charge_V,CAP_U_ADC_DATA.lowflt_out,set_U);
		   //电流和电压取最小值			 
			 if(CC_MODE_PWM<CV_MODE_PWM)
			 {
				 pid_pwm_out=CC_MODE_PWM;
			 }
			 else
			 {
				 pid_pwm_out=CV_MODE_PWM;
			 }
}
CCMRAM void CAP_U_I_CTRL_C(void)
{
	   //set_I=1000;
	   //set_U=10000;
		 set_I=PID_Incremental_lite(&CAP_charge_V,CAP_U_ADC_DATA.lowflt_out,set_U);
	   //set_I=-3500;
     CC_MODE_PWM=PID_Incremental_lite(&CAP_charge_I,CAP_I_ADC_DATA.lowflt_out,set_I);
	   pid_pwm_out=CC_MODE_PWM;
}
//单个电压环
CCMRAM void CAP_I_CTRL(void)
{
	 if(HRTIM_BIT==ON)//hrtim开启，电流环开始工作
	 {
		 //恒定电流测试
	   //set_I=1000;
     pid_pwm_out=PID_Incremental_lite(&CAP_charge_I,CAP_I_ADC_DATA.lowflt_out,set_I);//100
   }
	 else
	 {
	    CAP_charge_I.out=0; //不能让输出提前增加
	 }
}
//单个电流环
CCMRAM void CAP_U_CTRL(void)
{ 
	if(HRTIM_BIT==ON)//hrtim开启，电压环开始工作
	 {
		 //恒定电压测试
	   //set_U=7000;
     set_I=PID_Position(&CAP_charge_V,CAP_U_ADC_DATA.lowflt_out,set_U);//100
   }
	 else
	 {
	    CAP_charge_V.out=0; //不能让输出提前增加
	 }
	
}
CCMRAM void CAP_P_CTRL(void)
{
   if(HRTIM_BIT==ON)//hrtim开启，功率环开始工作
	 {
	   //set_P=10;
		 //set_I=set_P*1000.0f*1000.0f/CAP_U_ADC_DATA.lowflt_out;
     set_I=PID_Incremental_lite(&CAP_charge_P,CAP_P,set_P);
		 //set_I=CTRL_P*1000.0f/CAP_U_ADC_DATA.lowflt_out;
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
		 set_P=POWER_MAX+POW_P;   //电容当前功率=电源输出能力上限-当前底盘输出功率
		 if(set_P<(-200)){set_P=-200;}//最大输出功率30w
		 if(set_P > 200){set_P=200;}//最大充电功率功率20w
   }
	 else
	 {
	    set_P=0; //不能让输出提前增加
	 }
}

CCMRAM void PID_CTRL()
{
//此处用于调试占空比	
	//pid_pwm_out占空比调节范围
	//0.03-0.97  降压
	//0.97-1.3升压
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
	//pow_ducy=PWM-10000;
	//MAX 46080
  HRTIM1->sMasterRegs.MCMP1R = (pow_ducy) / 2u;
  HRTIM1->sMasterRegs.MCMP2R = (2*HRTIM1->sMasterRegs.MPER - pow_ducy)/ 2u;
  HRTIM1->sMasterRegs.MCMP3R = (HRTIM1->sMasterRegs.MPER - cap_ducy) / 2u; 
  HRTIM1->sMasterRegs.MCMP4R = (HRTIM1->sMasterRegs.MPER + cap_ducy) / 2u;
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
	 protection_mode=POWER_PROTECTION();
		
	 //充放检测
	 CAP_P_SET();
	 CAP_P_CTRL();
	 CAP_U_I_CTRL();
	 PID_CTRL();
	 //pid运算
}

CCMRAM uint16_t cap_time_set=0; //电容充电功率环心跳计时器
CCMRAM void U_P_CTRL_1000HZ(void)
{
		 POWER_ANALYSE_1000HZ();
		 //上电，等电压电流趋向稳定在控功
		 if(cap_time_set>100)
		 {
		 CAP_P_SET();
		 CAP_P_CTRL();
		 }
		 else
		 {
			 cap_time_set++;
		 }
		 CAP_U_CTRL ();
}
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

//积分型pid在电源运行中，不使用时电源pid 容易过度积分导致输出电压偏高

//当电源难以供应电流时，电流环会超调，需要外加电压环加以限制

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






