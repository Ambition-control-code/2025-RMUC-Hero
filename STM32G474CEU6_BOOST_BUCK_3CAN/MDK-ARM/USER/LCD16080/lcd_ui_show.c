#include "lcd.h"
#include "main.h"

struct 
{
  uint8_t run_state;
	int16_t list_mode;
	
}UI_STATE;

void LCD_I_SHOW_OFFSET(uint8_t *KEY_SET);
void LCD_UI_SHOW_MAIN(uint8_t *KEY_SET);
void LCD_UI_SHOW_MAIN_LIST(uint8_t *KEY_SET);
void WORK_MOTE_SET_lIST(uint8_t *KEY_SET);

void (*SCREEN_SHOW)(uint8_t *KEY_SET);



void UI_init(void)
{
  SCREEN_SHOW=LCD_UI_SHOW_MAIN;
}
void UI_RUN(void)
{
  SCREEN_SHOW(&key_input_state);
}
int16_t num_limit(int16_t input,int16_t max,int16_t min)
{
   if(input>max)
	 {
	   return min;
	 }
	 else if(input<min)
	 {
	   return max;
	 }
   return input;
}
	
//主界面
void LCD_UI_SHOW_MAIN(uint8_t *KEY_SET)
{
	  if(UI_STATE.run_state==0)//第一次运行
		{
		  LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
			LCD_DrawRectangle(1, 72, 159, 79,BRRED);
			LCD_DrawLine(3,74,158,74,GREEN);
			LCD_DrawLine(3,75,158,75,GREEN);
			LCD_DrawLine(3,76,158,76,GREEN);

			LCD_ShowString(0,0, "PO_I:",LIGHTBLUE,BLACK,12,0);
			LCD_ShowString(0,12,"PO_U:",LIGHTBLUE,BLACK,12,0);
			LCD_ShowString(0,24,"CL_I:",LIGHTBLUE,BLACK,12,0);
			LCD_ShowString(80,0,"CP_I:",LIGHTBLUE,BLACK,12,0);
			LCD_ShowString(80,12,"CP_U:",LIGHTBLUE,BLACK,12,0);
			LCD_ShowString(0,36,"CP_P:",LIGHTBLUE,BLACK,12,0);
			LCD_ShowString(0,48,"PO_P:",LIGHTBLUE,BLACK,12,0);
			LCD_ShowString(0,60,"CL_P:",LIGHTBLUE,BLACK,12,0);
		
		  UI_STATE.run_state=1;
			UI_STATE.list_mode=0;
		}
		//有输入按键事件
		if(*KEY_SET!=0)
		{
			 if(*KEY_SET==KEY_UP)
			 {
				  SCREEN_SHOW=LCD_UI_SHOW_MAIN_LIST;
			    UI_STATE.run_state=0;
			 }
			 else if(*KEY_SET==KEY_DOWN)
			 {

			 }
			 else if(*KEY_SET==KEY_PRESS)
			 {
			 
			 }
		   *KEY_SET=0;
		}
  	//LCD_ShowString(0,0,"PO_I:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(32,0,"-",(POW_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowIntNum(40,0,(POW_I_ADC_DATA.lowflt_out>0?POW_I_ADC_DATA.lowflt_out:-POW_I_ADC_DATA.lowflt_out),5,WHITE,BLACK,12);
		
		//LCD_ShowString(0,12,"PO_U:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowIntNum(40,12,POW_U_ADC_DATA.lowflt_out,5,WHITE,BLACK,12);
		
		//LCD_ShowString(0,24,"CL_I:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(32,24,"-",(CLS_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowIntNum(40,24,(CLS_I_ADC_DATA.lowflt_out>0?CLS_I_ADC_DATA.lowflt_out:-CLS_I_ADC_DATA.lowflt_out),5,WHITE,BLACK,12);
		
		//LCD_ShowString(80,0,"CP_I:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(80+32,0,"-",(CAP_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowIntNum(120,0,CAP_I_ADC_DATA.lowflt_out>0?CAP_I_ADC_DATA.lowflt_out:-CAP_I_ADC_DATA.lowflt_out,5,WHITE,BLACK,12);
			
		//LCD_ShowString(80,12,"CP_U:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowIntNum(120,12,CAP_U_ADC_DATA.lowflt_out,5,WHITE,BLACK,12);

		//LCD_ShowString(0,36,"CP_P:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(0+32,36,"-",(CAP_P>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowFloatNum1(48,36,abs_user(CAP_P),5,WHITE,BLACK,12);
	
    //LCD_ShowString(0,48,"PO_P:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(0+32,48,"-",(POW_P>0?BLACK:WHITE),BLACK,12,0);
		LCD_ShowFloatNum1(48,48,abs_user(POW_P),5,WHITE,BLACK,12);
		
		//LCD_ShowString(0,60,"CL_P:",LIGHTBLUE,BLACK,12,0);
		LCD_ShowString(0+32,60,"-",(CLS_P>0?BLACK:WHITE),BLACK,12,0);
		
		//PPE
		LCD_ShowFloatNum1(48,60,abs_user(CLS_P),5,WHITE,BLACK,12);
		
		LCD_ShowIntNum(140,60,protection_mode,3,WHITE,BLACK,12);
	  if(protection_mode)
		{
		  LCD_ShowString(100,60,"P_P_E=",BRRED,BLACK,12,0);
			
			if(protection_mode==1||protection_mode==3)  //过压保护
			{
			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(500);
			  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(500);
			}
			if(protection_mode==5)
			{
			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(1000);
			  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(1000);
			}
			if(protection_mode==4||protection_mode==2) //过流保护
			{
			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(200);
			  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			  HAL_Delay(200);
			}
		}
}
void LCD_I_SHOW_OFFSET(uint8_t *KEY_SET)
{
	if(UI_STATE.run_state==0)//第一次运行
	{
		 LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
		 LCD_ShowString(0,0, ">",RED,BLACK,12,0);
		 LCD_ShowString(10,0,"BACK",YELLOW,BLACK,12,0);
		 LCD_ShowString(10,12,"PO_I_ZERO_OFFSET",GREEN,BLACK,12,0);
		 LCD_ShowString(10,24,"CP_I_ZERO_OFFSET",GREEN,BLACK,12,0);
		 LCD_ShowString(10,36,"CL_I_ZERO_OFFSET",GREEN,BLACK,12,0);
		 LCD_ShowString(10,48,"BACK",YELLOW,BLACK,12,0);
//			LCD_ShowString(80,0,"CP_I:",LIGHTBLUE,BLACK,12,0);
		
//			LCD_ShowString(80,12,"CP_U:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(0,36,"CP_P:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(0,48,"PO_P:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(0,60,"CL_P:",LIGHTBLUE,BLACK,12,0);
		
		  UI_STATE.run_state=1;
		  UI_STATE.list_mode=0;
	}
	//有输入按键事件
	if(*KEY_SET!=0)
	{
		 if(*KEY_SET==KEY_UP)
		 {
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",BLACK,BLACK,12,0);	
			 UI_STATE.list_mode=num_limit(--UI_STATE.list_mode,4,0);
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",RED,BLACK,12,0);			 
		 }
		 else if(*KEY_SET==KEY_DOWN)
		 {
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",BLACK,BLACK,12,0);	
			 UI_STATE.list_mode=num_limit(++UI_STATE.list_mode,4,0);
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",RED,BLACK,12,0);			
		 }
		 else if(*KEY_SET==KEY_PRESS)
		 {
			 if(UI_STATE.list_mode==0||UI_STATE.list_mode==4)
			 {
			 	  SCREEN_SHOW=LCD_UI_SHOW_MAIN_LIST;
			    UI_STATE.run_state=0;
			 }
			 else 
			 {
			 	  I_ZERO_DATA_SET();
			 }
		 }
		 *KEY_SET=0;
	}	
	LCD_ShowString(120,12,"-",(POW_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
	LCD_ShowIntNum(128,12,(POW_I_ADC_DATA.lowflt_out>0?POW_I_ADC_DATA.lowflt_out:-POW_I_ADC_DATA.lowflt_out),5,WHITE,BLACK,12);
	
	LCD_ShowString(120,24,"-",(CAP_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
	LCD_ShowIntNum(128,24,CAP_I_ADC_DATA.lowflt_out>0?CAP_I_ADC_DATA.lowflt_out:-CAP_I_ADC_DATA.lowflt_out,5,WHITE,BLACK,12);
	
	LCD_ShowString(120,36,"-",(CLS_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
	LCD_ShowIntNum(128,36,(CLS_I_ADC_DATA.lowflt_out>0?CLS_I_ADC_DATA.lowflt_out:-CLS_I_ADC_DATA.lowflt_out),5,WHITE,BLACK,12);
		
}
void LCD_UI_SHOW_MAIN_LIST(uint8_t *KEY_SET)
{
	if(UI_STATE.run_state==0)//第一次运行
	{
		 LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
		 LCD_ShowString(0,0, ">",RED,BLACK,12,0);
		 LCD_ShowString(10,0,"BACK",YELLOW,BLACK,12,0);
		 LCD_ShowString(10,12,"I_OFFSET_SET",GREEN,BLACK,12,0);
		 LCD_ShowString(10,24,"U_OFFSET_SET",GREEN,BLACK,12,0);
		 LCD_ShowString(10,36,"WROK_MODE_SET",GREEN,BLACK,12,0);
		 LCD_ShowString(10,48,"CHASSIS_ZERO",GREEN,BLACK,12,0);
		 LCD_ShowString(10,60,"BACK",YELLOW,BLACK,12,0);
//			LCD_ShowString(80,0,"CP_I:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(80,12,"CP_U:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(0,36,"CP_P:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(0,48,"PO_P:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(0,60,"CL_P:",LIGHTBLUE,BLACK,12,0);
		
		  UI_STATE.run_state=1;
		  UI_STATE.list_mode=0;
	}
	//LCD_ShowString(32,,"-",(POW_I_ADC_DATA.lowflt_out>0?BLACK:WHITE),BLACK,12,0);
	//LCD_ShowIntNum(120,48,GMIBAL_YAW.real_angle,5,WHITE,BLACK,12);	
	//有输入按键事件
	if(*KEY_SET!=0)
	{
		 if(*KEY_SET==KEY_UP)
		 {
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",BLACK,BLACK,12,0);	
			 UI_STATE.list_mode=num_limit(--UI_STATE.list_mode,5,0);
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",RED,BLACK,12,0);			 
		 }
		 else if(*KEY_SET==KEY_DOWN)
		 {
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",BLACK,BLACK,12,0);	
			 UI_STATE.list_mode=num_limit(++UI_STATE.list_mode,5,0);
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",RED,BLACK,12,0);			
		 }
		 else if(*KEY_SET==KEY_PRESS)
		 {
		   if(UI_STATE.list_mode==0||UI_STATE.list_mode==5)
			 {
			 	 SCREEN_SHOW=LCD_UI_SHOW_MAIN;
			   UI_STATE.run_state=0;
			 }
			 else if(UI_STATE.list_mode==1)
			 {
			   SCREEN_SHOW=LCD_I_SHOW_OFFSET;
			   UI_STATE.run_state=0;
			 }
			 else if(UI_STATE.list_mode==3)
			 {
			   SCREEN_SHOW=WORK_MOTE_SET_lIST;
				 UI_STATE.run_state=0;
			 }
			 else if(UI_STATE.list_mode==4)
			 {
			   
				 //FLASH_USER.GMIBAL_ZERO_FLASH_DATA=GMIBAL_YAW.real_angle;
				 WRITE_FLASH_DATA(&FLASH_USER);
		
			 }
			 
		 }
		 *KEY_SET=0;
	}
	
}
void WORK_MOTE_SET_lIST(uint8_t *KEY_SET)
{
	if(UI_STATE.run_state==0)//第一次运行
	{
		 LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
		 LCD_ShowString(0,0, ">",RED,BLACK,12,0);
		 LCD_ShowString(10,0,"BACK",YELLOW,BLACK,12,0);
		 LCD_ShowString(10,12,"CHASSIS_MODE",GREEN,BLACK,12,0);
		 LCD_ShowString(10,24,"CapOnly_MODE",GREEN,BLACK,12,0);
		 LCD_ShowString(10,36,"NULL",GREEN,BLACK,12,0);
		 LCD_ShowString(10,48,"BACK",YELLOW,BLACK,12,0);
//			LCD_ShowString(80,0,"CP_I:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(80,12,"CP_U:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(0,36,"CP_P:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(0,48,"PO_P:",LIGHTBLUE,BLACK,12,0);
//			LCD_ShowString(0,60,"CL_P:",LIGHTBLUE,BLACK,12,0);
		
		  UI_STATE.run_state=1;
		  UI_STATE.list_mode=0;
	}
	//有输入按键事件
	if(*KEY_SET!=0)
	{
		 if(*KEY_SET==KEY_UP)
		 {
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",BLACK,BLACK,12,0);	
			 UI_STATE.list_mode=num_limit(--UI_STATE.list_mode,4,0);
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",RED,BLACK,12,0);			 
		 }
		 else if(*KEY_SET==KEY_DOWN)
		 {
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",BLACK,BLACK,12,0);	
			 UI_STATE.list_mode=num_limit(++UI_STATE.list_mode,4,0);
			 LCD_ShowString(0,UI_STATE.list_mode*12, ">",RED,BLACK,12,0);			
		 }
		 else if(*KEY_SET==KEY_PRESS)
		 {
		   if(UI_STATE.list_mode==0||UI_STATE.list_mode==4)
			 {
			 	 SCREEN_SHOW=LCD_UI_SHOW_MAIN_LIST;
			   UI_STATE.run_state=0;
			 }
			 else if(UI_STATE.list_mode==1)
			 {
			    FLASH_USER.MODE_SET=FLASH_USER.MODE_SET&0xfffc;
					FLASH_USER.MODE_SET|=0x0000;
					WRITE_FLASH_DATA(&FLASH_USER);
			 }
			 else if(UI_STATE.list_mode==2)
			 {
    			FLASH_USER.MODE_SET=FLASH_USER.MODE_SET&0xfffc;
					FLASH_USER.MODE_SET|=0x0001;
					WRITE_FLASH_DATA(&FLASH_USER);
			 }
			 
		 }
		 *KEY_SET=0;
	}
	uint16_t set_state;
	set_state=FLASH_USER.MODE_SET&WORK_MODE_STATE;
  LCD_ShowString(10,12,"CHASSIS_MODE",set_state==0?LIGHTBLUE:GREEN,BLACK,12,0);
	LCD_ShowString(10,24,"CapOnly_MODE",set_state==1?LIGHTBLUE:GREEN,BLACK,12,0);
  LCD_ShowString(10,36,"NULL",set_state==2?LIGHTBLUE:GREEN,BLACK,12,0);
	
}


void SET_FLAOT(uint8_t *KEY_SET,int16_t * set_value)
{




}

