#include "power_measurement.h"

#define PI    3.14159265F // π
#define PI2   6.28318530F // 2π


uint16_t ADC1_DMA_BUFF[ADC1_DMA_BUFF_SIZE*ADC_NUM];   //ADC1采集的是电容段的数据 A0电流 A1电压
uint16_t ADC2_DMA_BUFF[ADC2_DMA_BUFF_SIZE*ADC_NUM];   //ADC2采集的是电源段的数据 B14电流 B12电压

typedef struct 
{
 float AVG [AVG_FLT_LEN];
 uint8_t len;
}AVG;

AVG AVG_I;
void AVG_INIT()
{
  AVG_I.len=0;
}
CCMRAM float ACG_FLT_OUT (AVG* AVG_DATA,float data)
{
    AVG_DATA->AVG[AVG_DATA->len]=data;  
   	AVG_DATA->len++;
	  AVG_DATA->len=((AVG_DATA->len)> AVG_FLT_LEN)? 0:(AVG_DATA->len);
	  float add=0;
	  for(uint8_t i=0;i<AVG_FLT_LEN;i++)
	  {
	    add+=AVG_DATA->AVG[i];
	  }
		add/=AVG_FLT_LEN;
    return add;
}
float test_data(float in)
{
   return ACG_FLT_OUT(&AVG_I,in);

}

//200k频率 1周期=5000ns
//ADC参考电源位3V，0-4096对应0-3v
//采样时间+12.5个周期   设定2.5cylces情况下  采样时间为2.5+12.5=15个周期   ADC时钟频率为170MHZ/4=42.5MHZ  32个周期时15*（1/42500000）s=352ns，在400K的一周期时间内
//一共有两个量需要采集，使用dma连续采集2次是700ns一轮，电流环预定10Khz



CCMRAM  ADC_DATA CAP_I_ADC_DATA;
CCMRAM  ADC_DATA CAP_U_ADC_DATA;
CCMRAM  ADC_DATA POW_I_ADC_DATA;
CCMRAM  ADC_DATA POW_U_ADC_DATA;

//整体采集数据呈现
CCMRAM  float  CAP_I;
CCMRAM  float  CAP_U;  //电容电压电流

CCMRAM  float  POW_I;
CCMRAM  float  POW_U;  //电源电压电流

CCMRAM  float  CAP_P;
CCMRAM  float  POW_P;  //电源，电容功率

float  DC_DC_E; //效率计算


void ADC_INIT()
{	
	ADC_DATA_FLITER_INIT();
  //校准
	HAL_Delay(100);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_Delay(100);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_DMA_BUFF,ADC2_DMA_BUFF_SIZE*ADC_NUM );
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_DMA_BUFF,ADC1_DMA_BUFF_SIZE*ADC_NUM );
}


//低通滤波器参数初始化
void low_filter_init(ADC_DATA *DATA, float sample_f, float cutoff_f)
{
  float Tc; //时间常数τ
  float Fc = cutoff_f;//1000 截止频率
  float Fs = sample_f;//10000 采样频率
  if (Fc <= 0.0f || Fs <= 0.0f)
  {
    DATA->ka = 1.0f;
    DATA->kb = 0.0f;
    return;
  }

  Tc = 1.0f / (PI2 * Fc);

  DATA->ka = 1.0f / (1.0f + Tc * Fs);
  DATA->kb = 1.0f - DATA->ka;
}
//低通滤波参数初始化
void ADC_DATA_FLITER_INIT(void)
{
	
  low_filter_init(&POW_I_ADC_DATA,10000, 500);
	low_filter_init(&CAP_I_ADC_DATA,10000, 500);
	low_filter_init(&POW_U_ADC_DATA,10000, 500);
	low_filter_init(&CAP_U_ADC_DATA,10000, 500);
	
}
//对adc计算数据使用低通滤波
CCMRAM void ADC_DATA_FLITER(ADC_DATA *p)
{
	  p->lowflt_in=p->com_data;
    if (p->lowflt_out == p->lowflt_in)
    return;

    p->lowflt_out = p->ka * p->lowflt_in + p->kb * p->lowflt_out;

}
CCMRAM void ADC_DATA_AVG(uint16_t *ADC_DMA_BUFF,uint16_t *ADC_I,uint16_t *ADC_U,uint8_t ADC_LENGTH)
{
   u8 i;
	uint16_t A_I=0;
	uint16_t A_U=0;
	for(i=0;i<ADC_LENGTH;i++)
	{		  
 	   A_I+=ADC_DMA_BUFF[i*2];
		 A_U+=ADC_DMA_BUFF[(i*2)+1];	
	}
	*ADC_I= A_I/ADC_LENGTH;
	*ADC_U= A_U/ADC_LENGTH;
	return ;
}
 //把数据从dma里拆分出来
CCMRAM void MAKE_DMA_TO_ADC(uint16_t *ADC_DMA_BUFF,ADC_DATA *DATA1,ADC_DATA *DATA2,uint8_t ADC_LENGTH) //把数据从dma里拆分出来
{
	u8 i;
	for(i=0;i<ADC_LENGTH;i++)
	{	
     DATA1->adc_source_data[i]=ADC_DMA_BUFF[i*2];	
		 DATA2->adc_source_data[i]=ADC_DMA_BUFF[i*2+1];	
	}
	return;
}
	
//求平均值
CCMRAM float DATA_AVG(uint16_t *data_list,uint8_t LENGTH)
{
   uint8_t i=0;
	 uint32_t AVG;
	 float out;
   for(i=0;i<LENGTH;i++)
	 {
		 AVG+=data_list[i];
	 }
	 out=AVG/LENGTH;
	 
	 return out;
	 
}
//冒泡排序算法
CCMRAM void MUBBLING_SORTING(uint16_t * data,uint8_t length)
{
	 
   uint16_t min_data;
   uint8_t i=length;
   uint8_t j=0;
   for(i=length;i>1;i--)
   {
	     for(j=0;j<i-1;j++)
       {
			    if(data[j]>data[j+1])
          {  				
					   min_data=data[j+1];
						 data[j+1]=data[j];
						 data[j]=min_data; 
					}
			 }		 
	 }  

}
//寻找极限值算法
CCMRAM void FIND_MAX_MIN(uint16_t * data,uint8_t length,uint8_t num)
{
  uint16_t MAX=0,MIN=0;
	uint8_t i=0;
	uint8_t j=0;
	for(i=0 ;i<num;i++)
  {
		for(j=i;j<(length-i);j++)
		{
		     if(data[j]>data[length-i])
         {  				
					   MAX=data[length-i];
						 data[length-i]=data[j];
						 data[j]=MAX; 
				 }
				 if(data[j]<data[i])
         {  				
					   MIN=data[i];
						 data[i]=data[j];
						 data[j]=MIN; 
				 }
		}
	}
  return;
}
//中位平均滤波  n抛弃首尾值的个数
CCMRAM float middle_flt(uint16_t * data,uint8_t length,uint8_t n)
{
   uint32_t num=0; 
	 uint8_t i=n;
   for(;i<length-n;i++)
	{
	   num+=data[i];
	}
	return ((float)num)/(length-2*n);
}
//对acd采集数据执行中位平均滤波
CCMRAM void ADC_GET_MIDDLE_FLT_NUM(void)
{ 
  //先排序
  MUBBLING_SORTING(CAP_I_ADC_DATA.adc_source_data,ADC_NUM);
	MUBBLING_SORTING(CAP_U_ADC_DATA.adc_source_data,ADC_NUM);
	MUBBLING_SORTING(POW_I_ADC_DATA.adc_source_data,ADC_NUM);
	MUBBLING_SORTING(POW_U_ADC_DATA.adc_source_data,ADC_NUM);
	//在滤波
	CAP_I_ADC_DATA.flt_data=middle_flt(CAP_I_ADC_DATA.adc_source_data,ADC_NUM,2);
  CAP_U_ADC_DATA.flt_data=middle_flt(CAP_U_ADC_DATA.adc_source_data,ADC_NUM,2);
	POW_I_ADC_DATA.flt_data=middle_flt(POW_I_ADC_DATA.adc_source_data,ADC_NUM,2);
	POW_U_ADC_DATA.flt_data=middle_flt(POW_U_ADC_DATA.adc_source_data,ADC_NUM,2);
	
}
CCMRAM void ADC_I_GET_MIDDLE_FLT_NUM(void)
{
   //先排序
   //MUBBLING_SORTING(CAP_I_ADC_DATA.adc_source_data,ADC_NUM); 
   //MUBBLING_SORTING(POW_I_ADC_DATA.adc_source_data,ADC_NUM);	
	 FIND_MAX_MIN(CAP_I_ADC_DATA.adc_source_data,ADC_NUM ,2); 
	 FIND_MAX_MIN(POW_I_ADC_DATA.adc_source_data,ADC_NUM ,2); 
   //在滤波
	 CAP_I_ADC_DATA.flt_data=middle_flt(CAP_I_ADC_DATA.adc_source_data,ADC_NUM,2);
	 POW_I_ADC_DATA.flt_data=middle_flt(POW_I_ADC_DATA.adc_source_data,ADC_NUM,2);	

}
CCMRAM void ADC_U_GET_MIDDLE_FLT_NUM(void)
{
   //先排序
   //MUBBLING_SORTING(CAP_U_ADC_DATA.adc_source_data,ADC_NUM);
   //MUBBLING_SORTING(POW_U_ADC_DATA.adc_source_data,ADC_NUM);
	 FIND_MAX_MIN(CAP_U_ADC_DATA.adc_source_data,ADC_NUM ,2); 
	 FIND_MAX_MIN(POW_U_ADC_DATA.adc_source_data,ADC_NUM ,2); 
   //在滤波
	 CAP_U_ADC_DATA.flt_data=middle_flt(CAP_U_ADC_DATA.adc_source_data,ADC_NUM,2);
	 POW_U_ADC_DATA.flt_data=middle_flt(POW_U_ADC_DATA.adc_source_data,ADC_NUM,2);

}




CCMRAM void I_REAL_TRANSLATE()
{
//   POW_U=(float)POW_U_ADC*POW_U_magnification;
//   CAP_U=(float)CAP_U_ADC*CAP_U_magnification;
//	 POW_I=(float)(POW_I_common-POW_I_ADC)*POW_I_magnification;
//	 CAP_I=(float)(CAP_I_ADC-CAP_I_common)*CAP_I_magnification;
//	 POW_I=FLT_COM(&FLT_I_POW,POW_I,0.05);
//	 CAP_I=FLT_COM(&FLT_I_CAP,CAP_I,0.05);
	   POW_I_ADC_DATA.com_data=(float)((float)POW_I_common-(float)POW_I_ADC_DATA.flt_data)*POW_I_magnification;
	   CAP_I_ADC_DATA.com_data=(float)((float)CAP_I_ADC_DATA.flt_data-(float)CAP_I_common)*CAP_I_magnification;
}
CCMRAM void U_REAL_TRANSLATE()
{
   //POW_U=(float)POW_U_ADC*POW_U_magnification;
   //CAP_U=(float)CAP_U_ADC*CAP_U_magnification;
	
	 POW_U_ADC_DATA.com_data=(float)POW_U_ADC_DATA.flt_data*POW_U_magnification;
	 CAP_U_ADC_DATA.com_data=(float)CAP_U_ADC_DATA.flt_data*CAP_U_magnification;
	
   //POW_I=(float)(POW_I_common-POW_I_ADC)*POW_I_magnification;
   //CAP_I=(float)(CAP_I_ADC-CAP_I_common)*CAP_I_magnification;
}
//低通滤波平滑一下
CCMRAM void low_flt(void)
{
   ADC_DATA_FLITER(&POW_I_ADC_DATA);
	 ADC_DATA_FLITER(&POW_U_ADC_DATA);
	 ADC_DATA_FLITER(&CAP_I_ADC_DATA);
	 ADC_DATA_FLITER(&CAP_U_ADC_DATA);
}
CCMRAM void low_flt_I(void)
{
  ADC_DATA_FLITER(&POW_I_ADC_DATA);
	ADC_DATA_FLITER(&CAP_I_ADC_DATA);
}
	
CCMRAM void low_flt_U(void)
{
   ADC_DATA_FLITER(&POW_U_ADC_DATA);
	 ADC_DATA_FLITER(&CAP_U_ADC_DATA);
}

CCMRAM void GET_all_data()
{
       CAP_I=CAP_I_ADC_DATA.lowflt_out/1000.0f;
       CAP_U=CAP_U_ADC_DATA.lowflt_out/1000.0f;  //电容电压电流  单位V

       POW_I=POW_I_ADC_DATA.lowflt_out/1000.0f;
       POW_U=POW_U_ADC_DATA.lowflt_out/1000.0f;  //电源电压电流  单位I

       CAP_P=CAP_I*CAP_U;
       POW_P=POW_I*POW_U;   //电源，电容功率
	
	     if(CAP_I_ADC_DATA.lowflt_out<0) //电容电流方向为负数，证明电容正在充电
			 {
	        //DC_DC_E=abs_user(CAP_P)/abs_user(POW_P);
			 }
}

//电流电压分析程序
CCMRAM void POWER_ANALYSE_10000HZ(void)  //10khz
{
	//先将DMA里的ADC数据分好
	MAKE_DMA_TO_ADC(ADC1_DMA_BUFF,&POW_U_ADC_DATA,&CAP_U_ADC_DATA,ADC_NUM);
	MAKE_DMA_TO_ADC(ADC2_DMA_BUFF,&POW_I_ADC_DATA,&CAP_I_ADC_DATA,ADC_NUM);
	//再对acd采集数据执行中位平均滤波CAP_I_ADC_DATACAP_I_ADC_DATA
  //ADC_GET_MIDDLE_FLT_NUM();
	//然后计算出对应单位mv ma
	ADC_I_GET_MIDDLE_FLT_NUM();  //电流环只解算电流
	  
	I_REAL_TRANSLATE();
  //U_REAL_TRANSLATE();
	//计算的实际电压电流数据低通滤波一下
	low_flt_I();
	//统计一下电压电流功率
  return ;
}
CCMRAM void POWER_ANALYSE_ALL(void)
{
  //先将DMA里的ADC数据分好
	MAKE_DMA_TO_ADC(ADC1_DMA_BUFF,&POW_U_ADC_DATA,&CAP_U_ADC_DATA,ADC_NUM);
	MAKE_DMA_TO_ADC(ADC2_DMA_BUFF,&POW_I_ADC_DATA,&CAP_I_ADC_DATA,ADC_NUM);
	//再对acd采集数据执行中位平均滤波CAP_I_ADC_DATACAP_I_ADC_DATA
  //ADC_GET_MIDDLE_FLT_NUM();
	//然后计算出对应单位mv ma
	ADC_I_GET_MIDDLE_FLT_NUM();  
	ADC_U_GET_MIDDLE_FLT_NUM();  
	  
	I_REAL_TRANSLATE();
  U_REAL_TRANSLATE();
	//计算的实际电压电流数据低通滤波一下
	low_flt_I();
	low_flt_U();
	//统计一下电压电流功率
	GET_all_data();
  return ;
}

CCMRAM void  POWER_ANALYSE_1000HZ(void)
{
	
	 ADC_U_GET_MIDDLE_FLT_NUM();  //1k环只解算电压
	 U_REAL_TRANSLATE();
	 low_flt_U();
   GET_all_data();

}








