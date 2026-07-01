#include "UmpireDraw.h"

#define   fh    5      //frame_header 帧头数据长度
#define   ci    2      //cmd_id       cmdid长度
#define   crc16 2      //CRC16        crc16长度
//DMA发送完成标志
uint8_t DRAW_DMA_OK_STATE=1;
//统一发送缓冲
u8 fachu[128];

//单字节串口发送程序
//tou为发送数组的地址
//length为数组长度
uint16_t usart3_tx_max_time;
void SendData (u8 *tou,int length)
{ 
	  //等待dma发送完成中断把他写为1
	  if(DRAW_DMA_OK_STATE==0)
		{
			 usart3_tx_max_time++;
			 if(usart3_tx_max_time>100)
			 {
			   usart3_tx_max_time=0;
				 DRAW_DMA_OK_STATE=1;
			 }
			 return;
		}
		usart3_tx_max_time=0;
	  DRAW_DMA_OK_STATE=0;
	  //等待串口发送完成
	  while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC!=1));
	  HAL_UART_Transmit_DMA(&huart3,tou,length);
		return ;
}

//为发送的一帧数据添加crc校验
//在数组的第5个字节添加CRC8校验码，最后两个字节添加CRC16校验码
//tou为发送数组的地址
//length为数组长度
void addCRC(u8* tou,int length)
{
	  Append_CRC8_Check_Sum(tou,5);
	  Append_CRC16_Check_Sum(tou,length);
}

//copy复制函数
//tou 要复制的地址
//mubiao  复制的地
//changdu 复制字节数
void copy(u8* tou,u8* mubiao,int changdu)
{
	int i=0;
  while(changdu>i)
	{
	  tou[i]=mubiao[i];
		i++;
	}
}
//drawOnePicture  绘制一个图形函数
//ID              指令ID
//sendID          发送者ID
//receptionID     接收者ID
//PICTURE         图形数据
u8 bao=0;  
void drawOnePicture(u16 ID,u16 sendID,u16 receptionID,u8* PICTURE)
{
       
	  draw1picture one; 
	  //u8 fachu[30];//定义发送变量
	  one.SOF=0xA5;                         //添加帧头0XA5
	  one.data_length=0x0015;                //定义数据帧长度
	  one.seq=bao++;                         //包名
	  one.CRC8=0;                            //CRC8帧头校验
	  one.cmd_id= RtoR;                      //通信指令机器人与机器人
	  one.ID = ID;                           //指令ID
	  one.sendID=sendID;                     //发送者ID
	  one.receptionID=receptionID;           //接收者ID
	 
    one.CRC16=0;                           //CRC16校验
	
	  fachu[0]=0xA5;
	  copy(&fachu[1],(u8*)&one.data_length,2);
	  fachu[3]=one.seq;
	  copy(&fachu[fh],(u8*)&one.cmd_id,2);
	  copy(&fachu[fh+2],(u8*)&one.ID,2);
	  copy(&fachu[fh+4],(u8*)&one.sendID,2);
	  copy(&fachu[fh+6],(u8*)&one.receptionID,2);
    copy(&fachu[fh+8],PICTURE+1,15);
		 
    addCRC(fachu,30);             //添加校验码
	  SendData(fachu,30);           //打包发送
}


//deletePicture   删除一个图形函数
//ID              指令ID
//sendID          发送者ID
//receptionID     接收者ID
//pictureMode     图像操作
//screen          操作图层

void deletePicture(u16 ID,u16 sendID,u16 receptionID,u8 pictureMode,u8 screen)
{
   deletepicture one;
	 //u8 fachu[17];
	 one.SOF=0xA5;                         //添加帧头0XA5
	 one.data_length=0x08;                  //定义数据帧长度
	 one.seq=bao++;                         //包名
	 one.CRC8=0;                            //CRC8帧头校验
	 one.cmd_id= RtoR;                      //通信指令机器人与机器人
	 one.ID = ID;                           //指令ID
	 one.sendID=sendID;                     //发送者ID
	 one.receptionID=receptionID;           //接收者ID
	 one.operate_tpye=pictureMode;  
   one.layer=screen;	
   one.CRC16=0;  
   
	 fachu[0]=0xA5;
	 copy(&fachu[1],(u8*)&one.data_length,2);
	 fachu[3]=one.seq;
	 copy(&fachu[fh],(u8*)&one.cmd_id,2);
	 copy(&fachu[fh+2],(u8*)&one.ID,2);
	 copy(&fachu[fh+4],(u8*)&one.sendID,2);
	 copy(&fachu[fh+6],(u8*)&one.receptionID,2);
	 copy(&fachu[fh+8],(u8*)&one.operate_tpye,1);
   copy(&fachu[fh+9],(u8*)&one.layer,1);
   
	 addCRC(fachu,17);             //添加校验码
	 SendData(fachu,17);           //打包发送
}
//drawOnePicture  绘制两个图形函数
//ID              指令ID
//sendID          发送者ID
//receptionID     接收者ID
//PICTURE         图形数据
void drawTwoPicture(u16 ID,u16 sendID,u16 receptionID,u8 *PICTURE)
{
     //u8 fachu[60];//定义发送变量  
	   draw1picture one; 
	   one.SOF=0xA5;                         //添加帧头0XA5
	   
	   one.data_length=0x0024;                //定义数据帧长度
	   one.seq=bao++;                         //包名
	   one.CRC8=0;                            //CRC8帧头校验
	   one.cmd_id= RtoR;                      //通信指令机器人与机器人
	   one.ID = ID;                           //指令ID
	   one.sendID=sendID;                     //发送者ID
	   one.receptionID=receptionID;           //接收者ID
     one.CRC16=0;                           //CRC16校验

	
	   fachu[0]=0xA5;
	   copy(&fachu[1],(u8*)&one.data_length,2);
	   fachu[3]=one.seq;
	   copy(&fachu[fh],(u8*)&one.cmd_id,2);
	   copy(&fachu[fh+2],(u8*)&one.ID,2);
	   copy(&fachu[fh+4],(u8*)&one.sendID,2);
	   copy(&fachu[fh+6],(u8*)&one.receptionID,2);
     copy(&fachu[fh+8],PICTURE+1,15);
		 copy(&fachu[fh+23],PICTURE+1+16,15);
     addCRC(fachu,45);             //添加校验码
	   SendData(fachu,45);           //打包发送
}
//drawOnePicture  绘制五个图形函数
//ID              指令ID
//sendID          发送者ID
//receptionID     接收者ID
//PICTURE         图形数据
void drawFivePicture(u16 ID,u16 sendID,u16 receptionID,u8	*PICTURE)
{
     //u8 fachu[90];  
	   draw1picture one; 
	   one.SOF=0xA5;                          //添加帧头0XA5
	                            //定义发送变量
	   one.data_length=0x0051;                //定义数据帧长度
	   one.seq=bao++;                         //包名
	   one.CRC8=0;                            //CRC8帧头校验
	   one.cmd_id= RtoR;                      //通信指令机器人与机器人
	   one.ID = ID;                           //指令ID
	   one.sendID=sendID;                     //发送者ID
	   one.receptionID=receptionID;           //接收者ID
     one.CRC16=0;                           //CRC16校验

	   fachu[0]=0xA5;
	   copy(&fachu[1],(u8*)&one.data_length,2);
	   fachu[3]=one.seq;
	   copy(&fachu[fh],(u8*)&one.cmd_id,2);
	   copy(&fachu[fh+2],(u8*)&one.ID,2);
	   copy(&fachu[fh+4],(u8*)&one.sendID,2);
	   copy(&fachu[fh+6],(u8*)&one.receptionID,2);
     copy(&fachu[fh+8],PICTURE+1,15);
		 copy(&fachu[fh+23],PICTURE+1+16,15);
		 copy(&fachu[fh+38],PICTURE+1+32,15);
		 copy(&fachu[fh+53],PICTURE+1+48,15);
		 copy(&fachu[fh+68],PICTURE+1+64,15);
     addCRC(fachu,90);             //添加校验码
	   SendData(fachu,90);           //打包发送
}
//drawOnePicture  绘制七个图形函数
//ID              指令ID
//sendID          发送者ID
//receptionID     接收者ID
//PICTURE         图形数据
void drawSevenPicture(u16 ID,u16 sendID,u16 receptionID,u8	*PICTURE)
{
     //u8 fachu[120];//定义发送变量   
	   draw1picture one; 
	   one.SOF=0xA5;                         //添加帧头0XA5
	   one.data_length=0x0024;                //定义数据帧长度
	   one.seq=bao++;                         //包名
	   one.CRC8=0;                            //CRC8帧头校验
	   one.cmd_id= RtoR;                      //通信指令机器人与机器人
	   one.ID = ID;                           //指令ID
	   one.sendID=sendID;                     //发送者ID
	   one.receptionID=receptionID;           //接收者ID
     one.CRC16=0;                           //CRC16校验

	
	   fachu[0]=0xA5;
	   copy(&fachu[1],(u8*)&one.data_length,2);
	   fachu[3]=one.seq;
	   copy(&fachu[fh],(u8*)&one.cmd_id,2);
	   copy(&fachu[fh+2],(u8*)&one.ID,2);
	   copy(&fachu[fh+4],(u8*)&one.sendID,2);
	   copy(&fachu[fh+6],(u8*)&one.receptionID,2);
     copy(&fachu[fh+8], PICTURE+1+16*0,15);
		 copy(&fachu[fh+23],PICTURE+1+16*1,15);
		 copy(&fachu[fh+38],PICTURE+1+16*2,15);
		 copy(&fachu[fh+53],PICTURE+1+16*3,15);
		 copy(&fachu[fh+68],PICTURE+1+16*4,15);
		 copy(&fachu[fh+83],PICTURE+1+16*5,15);
		 copy(&fachu[fh+98],PICTURE+1+16*6,15);
     addCRC(fachu,120);             //添加校验码
	   SendData(fachu,120);           //打包发送
}

void send_(u16 ID,u16 sendID,u16 receptionID,u8* PICTURE)
{
       
	  draw1picture one; 
	  //u8 fachu[30];//定义发送变量
	  one.SOF=0xA5;                         //添加帧头0XA5
	  one.data_length=0x0015;                //定义数据帧长度
	  one.seq=bao++;                         //包名
	  one.CRC8=0;                            //CRC8帧头校验
	  one.cmd_id= RtoR;                      //通信指令机器人与机器人
	  one.ID = 0x308;                           //指令ID
	  one.sendID=sendID;                     //发送者ID
	  one.receptionID=receptionID;           //接收者ID
	 
    one.CRC16=0;                           //CRC16校验
	
	  fachu[0]=0xA5;
	  copy(&fachu[1],(u8*)&one.data_length,2);
	  fachu[3]=one.seq;
	  copy(&fachu[fh],(u8*)&one.cmd_id,2);
	  copy(&fachu[fh+2],(u8*)&one.ID,2);
	  copy(&fachu[fh+4],(u8*)&one.sendID,2);
	  copy(&fachu[fh+6],(u8*)&one.receptionID,2);
    copy(&fachu[fh+8],PICTURE+1,15);
		 
    addCRC(fachu,30);             //添加校验码
	  SendData(fachu,30);           //打包发送
}


void drawPicture(u16 ID,u16 sendID,u16 receptionID,u8 *PICTURE)
{
  switch (ID)
	{
		case draw1P:
		{drawOnePicture(ID,sendID,receptionID,PICTURE);break;}
		case draw2P:
		{drawTwoPicture(ID,sendID,receptionID,PICTURE);break;}
		case draw5P:
		{drawFivePicture(ID,sendID,receptionID,PICTURE);break;}
		case draw7P:
		{drawSevenPicture(ID,sendID,receptionID,PICTURE);break;}
		case 0x308:
		{drawSevenPicture(ID,sendID,receptionID,PICTURE);break;}
	}
}

