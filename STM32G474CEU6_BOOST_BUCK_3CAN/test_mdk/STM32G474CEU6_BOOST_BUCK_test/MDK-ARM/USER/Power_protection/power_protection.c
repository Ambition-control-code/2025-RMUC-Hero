#include "power_protection.h"
CCMRAM uint8_t HRTIM_BIT=0; //HRTIM定时器状态标志 0关闭，1开启

float abs_user(float in)
{
   if (in<0)
	 {
	   return -in;
	 }
   return  in;

}

CCMRAM uint8_t POWER_PROTECTION(void)
{
	
	 uint8_t pro_mode=0;
   //判断电容 电压，电流，电源 电压 电流是否超过保护值
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
	 if(abs_user(CAP_U_ADC_DATA.lowflt_out)<CAP_MIN_V+1000)
	 {
		  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
	 }
	 else
	 {
	    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
	 }


	 
	 
	 
	 if(pro_mode!=0)
	 {
      HRTIM_OFF();
			//HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
		  //HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
		  //HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);	
				 //DEBUG_TETST_SHOW	();	
		  HAL_TIM_Base_Stop_IT(&htim6);// 10khz
		  HAL_TIM_Base_Stop_IT(&htim7);// 1khz						
			
		}
//	 if(abs_user(CAP_U_ADC_DATA.lowflt_out)>abs_user(POW_U_ADC_DATA.lowflt_out))
//	 {
//	    return 5;
//	 }
   return pro_mode;
}


CCMRAM void HRTIM_OFF(void)
{
   HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //通道关闭
   HRTIM_BIT=OFF;
}

CCMRAM void HRTIM_ON(void)
{
   HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //通道开启 
   HRTIM_BIT=ON;
}
void DEBUG_TETST_SHOW(void)
{
	  LCD_DC_Set();
    LCD_CS_Set();
	
	  LCD_ShowString(0,0,"CL_I:",LIGHTBLUE,BLACK,12,0);
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




//上电时需要根据两边电压值，预估一个合适的PWM频率，避免炸板
void U_I_CTRL_INIT(void)
{
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_MASTER);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_D);	
	
	uint16_t time=0;
	float CYCLE;
	while(time<500)
	{
	  POWER_ANALYSE_ALL();
	  HAL_Delay(1);
		time++;
	}
	while(POWER_PROTECTION())
	{
	
	}
	//PID参数预整定
	if(CAP_U_ADC_DATA.lowflt_out>POW_U_ADC_DATA.lowflt_out)  //要升压开始//D=(Uout-Uin)/Uout
	{
		CYCLE=(CAP_U_ADC_DATA.lowflt_out-POW_U_ADC_DATA.lowflt_out)/CAP_U_ADC_DATA.lowflt_out;
		CAP_charge_I.out=(0.97f+CYCLE)*1.0f;
		CAP_charge_V.out=(0.97f+CYCLE)*1.0f;
		//set_U=CAP_U_ADC_DATA.lowflt_out*1.1f;
		set_P=0;
		HRTIM1->sMasterRegs.MCMP1R = (HRTIM1->sMasterRegs.MPER*0.97f) / 2u;
		HRTIM1->sMasterRegs.MCMP2R = (2.0f*HRTIM1->sMasterRegs.MPER - HRTIM1->sMasterRegs.MPER*0.97f) / 2u;
		HRTIM1->sMasterRegs.MCMP3R = (HRTIM1->sMasterRegs.MPER - HRTIM1->sMasterRegs.MPER*CYCLE*1.0f) / 2u; 
		HRTIM1->sMasterRegs.MCMP4R = (HRTIM1->sMasterRegs.MPER + HRTIM1->sMasterRegs.MPER*CYCLE*1.0f) / 2u;
	}
	else  //要降压开始 降压拓步pwm占空比等于输入输出电压比
	{
		CYCLE=CAP_U_ADC_DATA.lowflt_out/POW_U_ADC_DATA.lowflt_out;
		if(CYCLE<0.3){CYCLE=0.3f;}
		if(CYCLE>1){CYCLE=0.3f;}
		CAP_charge_I.out=CYCLE*1.0f;   //稍微提高一下占空比，启动先充电
		CAP_charge_V.out=CYCLE*1.0f;   //稍微提高一下占空比，启动先充电
    //CAP_charge_P.out=CAP_U_ADC_DATA.lowflt_out*1.0f;
    //set_U=CAP_U_ADC_DATA.lowflt_out*1.0f;
		set_P=0;
		HRTIM1->sMasterRegs.MCMP1R = (HRTIM1->sMasterRegs.MPER*CAP_charge_I.out) / 2u;
		HRTIM1->sMasterRegs.MCMP2R = (2.0f*HRTIM1->sMasterRegs.MPER - HRTIM1->sMasterRegs.MPER*CAP_charge_I.out) / 2u;
		HRTIM1->sMasterRegs.MCMP3R = (HRTIM1->sMasterRegs.MPER - HRTIM1->sMasterRegs.MPER*0.03f) / 2u; 
		HRTIM1->sMasterRegs.MCMP4R = (HRTIM1->sMasterRegs.MPER + HRTIM1->sMasterRegs.MPER*0.03f) / 2u;
	}
  HRTIM_ON();
	
	HAL_TIM_Base_Start_IT(&htim6);// 10khz
	//HAL_TIM_Base_Start_IT(&htim7);// 1khz
	
}





