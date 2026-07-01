#include "power_protection.h"

#define GET_24_POWER_IN  HAL_GPIO_ReadPin(PO24_IN_GPIO_Port,PO24_IN_Pin)
uint16_t error_time[8];
CCMRAM uint8_t HRTIM_BIT=0; //HRTIM定时器状态标志 0关闭，1开启

CCMRAM float abs_user(float in)
{
   if (in<0)
	 {
	   return -in;
	 }
   return  in;

}

uint16_t work_time;
CCMRAM uint8_t POWER_PROTECTION(void)//旧版本
{	
	 uint8_t pro_mode=0;
   //判断电容 电压，电流，电源 电压 电流是否超过保护值
//	 if(GET_24_POWER_IN)
//	 {
//		  pro_mode= 114;
//	 }
	 if(abs_user(POW_U_ADC_DATA.lowflt_out)>POWER_V_limit)
	 {
	    pro_mode= 1;
	 }
	    
	 if(abs_user(POW_I_ADC_DATA.lowflt_out)>POWER_I_limit)
	 {
	    pro_mode= 2;
	 }
	   
	 if(abs_user(CAP_U_ADC_DATA.lowflt_out)>CAP_V_limit)
	 {
	    pro_mode= 3;
	 }
	
	 if(abs_user(CAP_I_ADC_DATA.lowflt_out)>CAP_I_limit)
	 {
	    pro_mode= 4;
	 }

	 if(pro_mode!=0)
	 {
	    protection_mode=pro_mode;
	 }
	 if(pro_mode!=0) //错误保护
	 {
      HRTIM_OFF();
			//HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);	
			//DEBUG_TETST_SHOW	();	
		  HAL_TIM_Base_Stop_IT(&htim6);// 10khz
		  HAL_TIM_Base_Stop_IT(&htim7);// 1khz						
			
	 }
//	 else  //自动恢复
//	 { 
//		  if(HRTIM_BIT==OFF)
//			{
//			  work_time++;
//			}
//			if(work_time>100)
//			{
//				work_time=0;
//				protection_mode=0;
//	      HRTIM_ON();
//	      HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
//		    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);	
//			}
//	 }
//	 if(abs_user(CAP_U_ADC_DATA.lowflt_out)>abs_user(POW_U_ADC_DATA.lowflt_out))
//	 {
//	    return 5;
//	 }
   return pro_mode;
}

//检测程序启动后通过返回值确定问题状态
//错误信息不能被冲掉

CCMRAM Power_err Power_error_state;

CCMRAM uint8_t POWER_PROTECTION_LITE(void)
{
	
   uint8_t pro_mode=0;
	 //电压电流检测
	 error_time[0]=(abs_user(POW_U_ADC_DATA.lowflt_out)>POWER_V_limit) ? ++error_time[0] : 0;
	 error_time[1]=(abs_user(POW_I_ADC_DATA.lowflt_out)>POWER_I_limit) ? ++error_time[1] : 0;
	 error_time[2]=(abs_user(CAP_U_ADC_DATA.lowflt_out)>CAP_V_limit)   ? ++error_time[2] : 0;
	 error_time[3]=(abs_user(CAP_I_ADC_DATA.lowflt_out)>CAP_I_limit)   ? ++error_time[3] : 0;
	 //
	 error_time[4]=(abs_user(POW_U_ADC_DATA.lowflt_out)>ADC_V_MAX) ? ++error_time[4] : 0;

	 //效率检测
	 //error_time[5]=(DCDC_Efficiency>1.4f||DCDC_Efficiency<0.6f) ? ++error_time[5] : 0;
	 //电容检测
	 //电容电流=电容容量*电容电压变化率。   电容电流最大20A，电容容量50/12=4.16f。  
	 //电容电压变化率最大为20/4.16=4.8v/s=0.0048v/ms=0.00048v/100us,电容变化率大于这个可认为超电严重错误
	 error_time[6]= (CAP_U_ERR>0.19f) ? ++error_time[6] : 0;
	 //裁判系统底盘供电
//	 if(GET_24_POWER_IN)
//	 { 
//		 //裁判系统底盘断电需要立即关闭
//		 Power_error_state.CHASSIS_POWER_STATE=1;
//	   pro_mode= 114;
//		 HRTIM_OFF();
//		 //HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
//		 HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
//		 HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);	
//		 protection_mode=pro_mode;
//		 return pro_mode;	
//	 }
	 //电压电流时长判断
	 if(error_time[0]>2000)
	 {
	   pro_mode= 1;
		 Power_error_state.POW_LIM_U_STATE=1;
	 }
	 if(error_time[1]>10)
	 {
	   pro_mode= 2;
		 Power_error_state.POW_LIM_I_STATE=1;
	 }
	 if(error_time[2]>10)
	 {
	   pro_mode= 3;
		 Power_error_state.CAP_LIM_U_STATE=1;
	 }
	 if(error_time[3]>10)
	 {
	   pro_mode= 4;
		 Power_error_state.CAP_LIM_I_STATE=1;
	 }
	 if(error_time[4]>150)
	 {
	   pro_mode= 5;
		 Power_error_state.POW_LIM_U_STATE=2;
	 }
	 if(error_time[5]>10)
	 {
	   pro_mode= 6;
		 Power_error_state.LOOP_EFF_STATE=1;
	 }
	 if(error_time[6]>20)
	 {
	   pro_mode= 7;
		 Power_error_state.CAP_ERR_STATE=1;
	 }
	 if(pro_mode!=0) //错误保护
	 {
		  protection_mode=pro_mode;
      HRTIM_OFF();
			//HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);	
			//DEBUG_TETST_SHOW	();	
		  //HAL_TIM_Base_Stop_IT(&htim6);// 10khz
		  //HAL_TIM_Base_Stop_IT(&htim7);// 1khz						
			
	 }
	 //protection_mode=pro_mode;
	 return pro_mode;	
}


CCMRAM void HRTIM_OFF(void)
{  
	 HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //通道关闭
	 //HAL_HRTIM_WaveformCountStop_IT(&hhrtim1,HRTIM_TIMERID_MASTER);
   HRTIM_BIT=OFF;
}

CCMRAM void HRTIM_ON(void)
{
   HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //通道开启 
	 //HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,HRTIM_TIMERID_MASTER);
   HRTIM_BIT=ON;
}


void DEBUG_TETST_SHOW(void)
{
	  LCD_DC_Set();
    LCD_CS_Set();
	
	  LCD_ShowString(0,0,"PO_I:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(32,0,"-",(POW_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowIntNum(40,0,(POW_I_ADC_DATA.lowflt_out>0?POW_I_ADC_DATA.lowflt_out:-POW_I_ADC_DATA.lowflt_out),5,WHITE,BLACK,12);
		
		LCD_ShowString(0,20,"DC_U:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowIntNum(40,20,POW_U_ADC_DATA.lowflt_out,5,WHITE,BLACK,12);
		
		LCD_ShowString(80,0,"CP_I:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(80+32,0,"-",(CAP_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowIntNum(120,0,abs_user(CAP_I_ADC_DATA.lowflt_out),5,WHITE,BLACK,12);
			
		LCD_ShowString(80,20,"CP_U:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowIntNum(120,20,CAP_U_ADC_DATA.lowflt_out,5,WHITE,BLACK,12);
	
	  LCD_ShowString(80,40,"ERROR_MODE",LIGHTBLUE,BLACK,12,0);
}

//尝试从错误中恢复过来
void U_I_CTRL_DEBUG()
{
	 //底盘断电
   if(Power_error_state.CHASSIS_POWER_STATE!=0)
	 { //检测到底盘上电且没有其他错误信息时,尝试恢复
//		 if((GET_24_POWER_IN==0)&&(Power_error_state.CAP_ERR_STATE==0))
//		 {
//		    Power_error_state.CHASSIS_POWER_STATE=0;
//			  U_I_CTRL_INIT();
//			  LED1_ON;
//	      LED2_ON;
//			  if(protection_mode==114)
//				{
//				  protection_mode=0;
//				}
//		 }
	 }
}



//上电时需要根据两边电压值，预估一个合适的PWM频率，避免炸板
void U_I_CTRL_INIT(void)
{
	HRTIM_OFF();
	//启动主定时器，adc分析工作，pwm输出不工作
	HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,HRTIM_TIMERID_MASTER);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_D);	
	
	//底盘控制代码在超电启动时应该关闭
	HAL_TIM_Base_Stop_IT(&htim6);// 1khz
	//HAL_TIM_Base_Stop_IT(&htim7);// 100hz
	uint16_t time=0;
	float CYCLE;
//	while(time<200)
//	{
//	  POWER_ANALYSE_ALL();
//	  HAL_Delay(1);
//		time++;
//	}
  //没有错误才能启动
	while(POWER_PROTECTION()){BEEP_ON;}
	//延时一会
	BEEP_OFF;
	
	while(time<200)
	{
		time++;
	  HAL_Delay(1);
	  POWER_ANALYSE_ALL();
		while(POWER_PROTECTION()){}
	}

	//目前最小占空比10%
	//PID参数预整定
	if(CAP_U_ADC_DATA.lowflt_out>POW_U_ADC_DATA.lowflt_out)  //要升压开始//D=(Uout-Uin)/Uout
	{
		CYCLE=(CAP_U_ADC_DATA.lowflt_out-POW_U_ADC_DATA.lowflt_out*0.9f)/CAP_U_ADC_DATA.lowflt_out;
		CAP_charge_I.out=(0.90f+CYCLE)*1.0f;
		CAP_charge_V.out=(0.90f+CYCLE)*1.0f;
		//set_U=CAP_U_ADC_DATA.lowflt_out*1.1f;
		set_P=0;	
		CAP_charge_P.out=1111;
	  HRTIM1->sMasterRegs.MCMP1R = (HRTIM1->sMasterRegs.MPER - HRTIM1->sMasterRegs.MPER*0.90f) / 2u;
		HRTIM1->sMasterRegs.MCMP2R = (HRTIM1->sMasterRegs.MPER + HRTIM1->sMasterRegs.MPER*0.90f) / 2u;
		HRTIM1->sMasterRegs.MCMP3R = (HRTIM1->sMasterRegs.MPER*CYCLE*1.0f) / 2u; 
		HRTIM1->sMasterRegs.MCMP4R = (2.0f*HRTIM1->sMasterRegs.MPER - HRTIM1->sMasterRegs.MPER*CYCLE*1.0f) / 2u;
	}
	else  //要降压开始 降压拓步pwm占空比等于输入输出电压比
	{
		CYCLE=(0.9f*CAP_U_ADC_DATA.lowflt_out)/POW_U_ADC_DATA.lowflt_out;
		if(CYCLE<0.1f){CYCLE=0.1f;}
		if(CYCLE>0.9f){CYCLE=0.9f;}
		CAP_charge_I.out=CYCLE*1.0f;   //稍微提高一下占空比，启动先充电
		CAP_charge_V.out=CYCLE*1.0f;   //稍微提高一下占空比，启动先充电
    //CAP_charge_P.out=CAP_U_ADC_DATA.lowflt_out*1.0f;
    //set_U=CAP_U_ADC_DATA.lowflt_out*1.0f;
		set_P=0;
		CAP_charge_P.out=1111;
		HRTIM1->sMasterRegs.MCMP1R = (HRTIM1->sMasterRegs.MPER - HRTIM1->sMasterRegs.MPER*CAP_charge_I.out) / 2u;
		HRTIM1->sMasterRegs.MCMP2R = (HRTIM1->sMasterRegs.MPER + HRTIM1->sMasterRegs.MPER*CAP_charge_I.out) / 2u;
		HRTIM1->sMasterRegs.MCMP3R = (HRTIM1->sMasterRegs.MPER*0.10f) / 2u; 
		HRTIM1->sMasterRegs.MCMP4R = (2.0f*HRTIM1->sMasterRegs.MPER - HRTIM1->sMasterRegs.MPER*0.10f) / 2u;
	}
	
	//启动环路前设置下效率系数
	//DCDC_Efficiency=0.85f;
	
  HRTIM_ON();
	//底盘控制，超电通信定时器中断（回调在u_i_pid.c）
	HAL_TIM_Base_Start_IT(&htim6);// 1khz
	HAL_TIM_Base_Start_IT(&htim7);// 100hz
	
}


//uint8_t  a=0;

//void HRTIM1_TIMB_IRQHandler(void)
//{
//  a++;
//  return ;
//}


//HAL_HRTIM_TIMxRegisterCallback





