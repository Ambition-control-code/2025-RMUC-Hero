#ifndef __POWERMEA_H
#define __POWERMEA_H
#include "main.h"
#define ADC1_DMA_BUFF_SIZE 2 //adc1采集通道数
#define ADC2_DMA_BUFF_SIZE 2 //adc2采集通道数
#define ADC_NUM            20 //连续采集数量

//0-4096=>0-3  U=adc_in*((1/4096)*3.3*10)=0.0146484375v=8.0566mv  理论   0-30v参考电压3.3v放大10倍0-33v
//0.008056640625v  
//后期校准时缩放系数会和理论略有差异
#define  POW_U_magnification  8.1166f//14.64/2//电源电压原始值对应实际值放大倍数 //后期更改了放大倍数，现在是10倍
#define  CAP_U_magnification  8.0391f//14.64/2//电容电压原始值对应实际值放大倍数

//U电阻=IR=I*4mr   UADC=adc_in*(1/4096)*3.3  UADC=U中点+U电阻*50    UADC-U中点=U电阻*50=1*0.004*50   0.0040283203125
//G474采样电阻6mr 使用ina240A1采样放大20倍  3.3v/2=1.65v  1.65v/20=0.0825V  I=U/R=0.0825V/0.006R=13.75A MAX    13.75/2048=0.0067138671875   0.0100708
//后期校准时缩放系数会和理论略有差异
#define  POW_I_magnification   10.0708f      //电源电流原始值对应实际值的放大倍数
#define  CAP_I_magnification   10.0708f      //电容电流原始值对应实际值的放大倍数

//1.65V   common=4096*(1.65/3)=2252.8
//后期校准时电压中位会和理论略有差异
#define  POW_I_common       2054.5f    //2048    //电源中点原始值
#define  CAP_I_common       2054.5f    //2048    //电源中点原始值

#define AVG_FLT_LEN 8

extern uint16_t ADC1_DMA_BUFF[ADC1_DMA_BUFF_SIZE*ADC_NUM];
extern uint16_t ADC2_DMA_BUFF[ADC2_DMA_BUFF_SIZE*ADC_NUM];

extern uint16_t CAP_I_ADC;
extern uint16_t CAP_U_ADC;
extern uint16_t POW_I_ADC;
extern uint16_t POW_U_ADC;

extern float  CAP_P;
extern float  POW_P;  //电源，电容功率

typedef struct 
{
	 //计算-滤波的最终值
   float com_data;   //mv ma
	 float lowflt_in;
	 float lowflt_out;
	 //adc dma 原始值   
	 uint16_t   adc_source_data[ADC_NUM];
	 //原始值滤波后的数据
	 float   flt_data;
	 //低通滤波数值
	 float ka;
	 float kb;
	  	 
}ADC_DATA;


extern  ADC_DATA CAP_I_ADC_DATA;
extern  ADC_DATA CAP_U_ADC_DATA;
extern  ADC_DATA POW_I_ADC_DATA;
extern  ADC_DATA POW_U_ADC_DATA;


void ADC_INIT(void);
void POWER_ANALYSE_1000HZ(void);
void ADC_DATA_AVG(uint16_t *ADC_DMA_BUFF,uint16_t *ADC_I,uint16_t *ADC_U,uint8_t ADC_LENGTH);
void I_REAL_TRANSLATE(void);
void U_REAL_TRANSLATE(void);
void POWER_ANALYSE_10000HZ(void);
void ADC_DATA_FLITER_INIT(void);
void POWER_ANALYSE_ALL(void);
float test_data(float in);
#endif

