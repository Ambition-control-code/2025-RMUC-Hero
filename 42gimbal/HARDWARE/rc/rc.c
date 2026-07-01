#include "rc.h"
#include "bsp_dwt.h"
#include "string.h"

#define RX_BUF_SIZE 18		//DMA接收缓冲区大小
 u8 RXBuff[RX_BUF_SIZE];		//DMA接收缓冲区
 u8 sendchassic[5],recieve_error;//2024.11.27新增

 RC rc; 
 
const  uint8_t PS2_cmnd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};   //请求获取数据命令                         
static uint8_t PS2_data[9] = {0};  //接收的数据
 
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern u8 dbus_recieve_1[36],dbus_recieve_2[36],VT13_recieve_1[42],VT13_recieve_2[42];
extern long int refree[9];

///////////////////////////////////////////////////////////
////IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     
#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO口操作
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 
#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 
#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入
#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入
#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入
#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入
//PS2手柄的输入输出口
#define DI()     PAin(6)      //数据输入引脚

#define CMD_H()   PAout(7)=1   //命令位高
#define CMD_L()   PAout(7)=0   //命令位低

#define CS_H()   PAout(8)=1    //CS拉高(别名ATT)
#define CS_L()   PAout(8)=0    //CS拉低(别名ATT)

#define CLK_H()  PAout(9)=1    //时钟拉高
#define CLK_L()  PAout(9)=0    //时钟拉低
/**
  * @简  述  PS2数据读写函数
  * @参  数  data:要写入的数据
  * @返回值  读出数据
  */
static uint8_t PS2_ReadWriteData(uint8_t data)
{
	uint8_t ref,res;
	
	
	for(ref = 0x01; ref > 0x00; ref <<= 1)
	{
		CLK_L();
		if(ref&data)
			CMD_H();
		else
			CMD_L();	
		
		DWT_DELAY_US(16);
		
		CLK_H();
		if(DI())
			res |= ref; 
		
		DWT_DELAY_US(16);		
	}
	
	CMD_H();

	//返回读出数据
    return res;	
}

/**
  * @简  述  PS2获取按键及摇杆数值。
  * @参  数  *JoystickStruct 手柄键值结构体
  * @返回值  无
  */
void AX_PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct)
{
	uint8_t i;
	
	//使能手柄
	CS_L();
	
	//读取PS2数据
	for(i=0; i<9; i++)
	{
		PS2_data[i] = PS2_ReadWriteData(PS2_cmnd[i]);
		DWT_DELAY_US(16);
	}
	
	//关闭使能
	CS_H();

	//数值传递
	JoystickStruct->mode = PS2_data[1];
	JoystickStruct->btn1 = ~PS2_data[3];
	JoystickStruct->btn2 = ~PS2_data[4];
	JoystickStruct->RJoy_LR = PS2_data[5];
	JoystickStruct->RJoy_UD = PS2_data[6];
	JoystickStruct->LJoy_LR = PS2_data[7];
	JoystickStruct->LJoy_UD = PS2_data[8];
}

///////////////////////////////////////////////////////////

void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);

}

u8 if_change(u8 flag,u8 last_flag)
{
if(flag!=last_flag) {return 1;}
else {return 0;}
}

u8 if_jump_up(u8 flag,u8 last_flag)
{
if(flag-last_flag==-1) {return 1;}
else {return 0;}
}

float keyboard_jump_flag_shift,keyboard_jump_flag_e,key_shift_last,key_e_last;

void deal_dbus(u8 *RXBuff)
{
	  refree[5]++;
	  rc.time=DWT_GetTimeline_ms();
		if(((RXBuff[0]| (RXBuff[1] << 8)) & 0x07ff)){
		rc.R_x = ((RXBuff[0]| (RXBuff[1] << 8)) & 0x07ff)-1024; 
		sendchassic[0]=RXBuff[0];
		sendchassic[1]=RXBuff[1];}
		if((((RXBuff[1] >> 3) | (RXBuff[2] << 5)) & 0x07ff)){
		rc.R_y = (((RXBuff[1] >> 3) | (RXBuff[2] << 5)) & 0x07ff)-1024;
		sendchassic[2]=RXBuff[2];} //!< Channel 1
		if((((RXBuff[2] >> 6) | (RXBuff[3] << 2) | (RXBuff[4] << 10))& 0x07ff)){
		rc.L_x = (((RXBuff[2] >> 6) | (RXBuff[3] << 2) | (RXBuff[4] << 10))& 0x07ff)-1024;}//!< Channel 2
		if((((RXBuff[4] >> 1) | (RXBuff[5] << 7)) & 0x07ff)){
		rc.L_y = (((RXBuff[4] >> 1) | (RXBuff[5] << 7)) & 0x07ff)-1024;} //!< Channel 3
		rc.sl = (((RXBuff[5] >> 4)& 0x000C) >> 2); //!< Switch left
		rc.sr = (((RXBuff[5] >> 4)& 0x0003)); //!< Switch left
		int16_t temp;
		temp = 1024 - (RXBuff[16]|(RXBuff[17] << 8));//遥控器侧面的拨轮
//		if(temp>30000) {rc.trigger=65535-temp;}
//		else {
		rc.trigger=temp;
//		}
		
		rc.sl_change_flag=if_change(rc.sl,rc.last_sl);
		rc.sr_change_flag=if_change(rc.sr,rc.last_sr);		
		
		rc.last_sl=rc.sl;
		rc.last_sr=rc.sr;

		rc.mouse_x = RXBuff[6] | (RXBuff[7] << 8); //!< Mouse X axis
		rc.mouse_y = RXBuff[8] | (RXBuff[9] << 8); //!< Mouse Y axis
		rc.mouse_z = RXBuff[10] | (RXBuff[11] << 8); //!< Mouse Z axis
		rc.mouse_l = RXBuff[12]; //!< Mouse Left Is Press ?
		rc.mouse_r = RXBuff[13]; //!< Mouse Right Is Press ?
		rc.key = RXBuff[14] | (RXBuff[15] << 8); //!< KeyBoard value
		rc.W = (rc.key&0x01)>>0;
		rc.S = (rc.key&0x02)>>1;
		rc.A = (rc.key&0x04)>>2;
		rc.D = (rc.key&0x08)>>3;
		rc.Shift = (rc.key&0x10)>>4;
		rc.Ctrl = (rc.key&0x20)>>5;
		rc.Q = (rc.key&0x40)>>6;
		rc.E = (rc.key&0x80)>>7;
		rc.R = (rc.key&0x100)>>8;
		rc.F = (rc.key&0x200)>>9;
		rc.G = (rc.key&0x400)>>10;
		rc.Z = (rc.key&0x800)>>11;
		rc.X = (rc.key&0x1000)>>12;
		rc.C = (rc.key&0x2000)>>13;
		rc.V = (rc.key&0x4000)>>14;
		rc.B = (rc.key&0x8000)>>15;
		//遥控器值抛出异常
		rc.R_x = rc.R_x > 660 ? 0 : rc.R_x;
		rc.R_x = rc.R_x < -660 ? 0 : rc.R_x;
		
		rc.R_y = rc.R_y > 660 ? 0 : rc.R_y;
		rc.R_y = rc.R_y < -660 ? 0 : rc.R_y;
		
		rc.L_x = rc.L_x > 660 ? 0 : rc.L_x;
		rc.L_x = rc.L_x < -660 ? 0 : rc.L_x;
		
		rc.L_y = rc.L_y > 660 ? 0 : rc.L_y;
		rc.L_y = rc.L_y < -660 ? 0 : rc.L_y;
		
		rc.mouse_x = rc.mouse_x > 1024 ? 0 : rc.mouse_x;
		rc.mouse_x = rc.mouse_x < -1024 ? 0 : rc.mouse_x;
		rc.mouse_y = rc.mouse_y > 1024 ? 0 : rc.mouse_y;
		rc.mouse_y = rc.mouse_y < -1024 ? 0 : rc.mouse_y;
}
								 

union{
VT13_data use_data;
u8  temp_data[21];
}VT13_USE;

void USER_UART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
  {
			__HAL_DMA_DISABLE(huart->hdmarx);

			huart->hdmarx->Instance->CR |= DMA_SxCR_CT;

		  if((VT13_recieve_1[0]==0xA9)&&(VT13_recieve_1[1]==0x53))
		  {memcpy(VT13_USE.temp_data,VT13_recieve_1,21);refree[8]++;}
			refree[7]++;
      __HAL_DMA_SET_COUNTER(huart->hdmarx,21*2);

  }
  else
  {
			__HAL_DMA_DISABLE(huart->hdmarx);

			huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
		
		  if((VT13_recieve_2[0]==0xA9)&&(VT13_recieve_2[1]==0x53))
		  {memcpy(VT13_USE.temp_data,VT13_recieve_2,21);refree[8]++;}
			
		  if((VT13_recieve_2[1]==0xA9)&&(VT13_recieve_2[2]==0x53))
		  {
			for(int i=0;i<20;i++)
			{
				VT13_USE.temp_data[i]=VT13_recieve_2[i+1];
			}
			refree[8]++;}
			refree[7]++;
      __HAL_DMA_SET_COUNTER(huart->hdmarx,21*2);

  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
if(huart->Instance == UART5)
	{		
		USER_UART5_RxHandler(huart,Size);	
	}

  /* reset the Reception Type */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}

//串口中断
void USART3_IRQHandler(void)
{
//    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart3);
//    }
//    else if(USART3->SR & UART_FLAG_IDLE)
	  if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 36 - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = 36;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == 18)
            {
		        HAL_UART_Receive_DMA(&huart3, (uint8_t*)dbus_recieve_1, 18);                                               
	         	deal_dbus(dbus_recieve_1);
            }
						else{recieve_error=1;}
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 36 - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = 36;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == 18)
            {
		HAL_UART_Receive_DMA(&huart3, (uint8_t*)dbus_recieve_2, 18);                                               
		deal_dbus(dbus_recieve_2);
            }
						 else{recieve_error=1;}
        }
    }

}

void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{	
  /* configuare the huart Reception Type TOIDLE */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

  /* configuare the huart Receicve Size */
   huart->RxXferSize = DataLength;
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* Disable DMA */
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(huart->hdmarx->Instance->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  huart->hdmarx->Instance->PAR = (uint32_t)SrcAddress;

  /* Configure the destination memory Buffer address */
  huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  huart->hdmarx->Instance->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}




