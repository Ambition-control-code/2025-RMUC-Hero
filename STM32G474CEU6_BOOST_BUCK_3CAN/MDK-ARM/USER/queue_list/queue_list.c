#include "queue_list.h" 
#include "string.h"

//自定义队列，使用半中断全中断空闲中断入队
//出队，完成数据解算并实现从首部到队尾部的转换。

CCMRAM queue USART1_RX_queue;
CCMRAM queue USART3_RX_queue;

void queue_rx_init(void)
{
     //usart_rx
     //初始化队首队尾地址
     USART1_RX_queue.front_add=0;
		 USART1_RX_queue.rear_add=0;
		 
		 //装填队列缓冲区地址
		 USART1_RX_queue.queue_list=USART1_RX_BUFF;
		 //装填缓冲区大小
		 USART1_RX_queue.list_size=USART1_RX_BUFF_LENGTH;
		 USART1_RX_queue.space_size=USART1_RX_BUFF_LENGTH;
		 //
		 USART1_RX_queue.data_byte_size=1;
	 
	   //usart_rx
     //初始化队首队尾地址
     USART3_RX_queue.front_add=0;
		 USART3_RX_queue.rear_add=0;
		 
		 //装填队列缓冲区地址
		 USART3_RX_queue.queue_list=USART3_RX_BUFF;
		 //装填缓冲区大小
		 USART3_RX_queue.list_size=USART3_RX_BUFF_LENGTH;
		 USART3_RX_queue.space_size=USART3_RX_BUFF_LENGTH;
		 //
		 USART3_RX_queue.data_byte_size=1;	 
}
 
 
 //入队操作输入数据地址，长度，进入队列名
 //输出1成功  非1失败
CCMRAM uint8_t queue_data_in(uint8_t *data,uint16_t size,queue* queue_list)
{
      if(queue_list->space_size<(size+queue_list->data_byte_size)) 
			{//如果队列剩余空间不够，存放失败
      			return 2;	
			}
			else
			{  
			   		//更新剩余空间
			      queue_list->space_size-=(size+queue_list->data_byte_size);
			      //从队列尾部存入数据
						if(queue_list->data_byte_size==1)
						{
						     queue_list->queue_list[queue_list->rear_add]=size;
						}
						else
						{
						      queue_list->queue_list[queue_list->rear_add]=size>>8;
									queue_list->queue_list[queue_list->rear_add+1]=size;
						}
						uint16_t add=0;
						queue_list->rear_add=(queue_list->rear_add+queue_list->data_byte_size)% queue_list->list_size;
				    while(size)
						{
						     queue_list->queue_list[queue_list->rear_add]=data[add];
						     queue_list->rear_add=(queue_list->rear_add+1)% queue_list->list_size;
								 add++;
								 size--;
						}
						
						return 1;
			}
			//return 0;
}
 //出队操作      输出数据地址,进入队列名
 //输出1成功  非1失败
CCMRAM uint8_t queue_data_out(uint8_t *data,queue* queue_list)
{
      if(queue_list->space_size>=queue_list->list_size) 
			{//如果队列为空，取出失败
      			return 2;	
			}
			else
			{  
			      uint16_t  data_size;
						if(queue_list->data_byte_size==1)
						{
						     data_size=queue_list->queue_list[queue_list->front_add];
						}
						else
						{
						     data_size=(((uint16_t)queue_list->queue_list[queue_list->front_add])<<8)+queue_list->queue_list[queue_list->rear_add+1];
						}
						//更新剩余空间
			      queue_list->space_size+=(data_size+queue_list->data_byte_size);
					  //从队列头部取出数据
						uint16_t add=0;
						queue_list->front_add=(queue_list->front_add+queue_list->data_byte_size)% queue_list->list_size;
						while(data_size)
						{
						      data[add]=queue_list->queue_list[queue_list->front_add];
						      queue_list->front_add=(queue_list->front_add+1)% queue_list->list_size;
								  add++;
								  data_size--;					
						}
						return 1;
			}
			//return 0;
}


//清除队列头部的数据，出队，丢弃队列头部的数据
 //输出1成功  非1失败
CCMRAM uint8_t queue_out(queue* queue_list)
{
       if(queue_list->space_size>=queue_list->list_size) 
			{//如果队列为空，取出失败
      			return 2;	
			}
			else
			{  
			      uint16_t  data_size;
						if(queue_list->data_byte_size==1)
						{
						     data_size=queue_list->queue_list[queue_list->front_add];
						}
						else
						{
						     data_size=(((uint16_t)queue_list->queue_list[queue_list->front_add])<<8)+queue_list->queue_list[queue_list->rear_add+1];
						}
						//更新剩余空间
			      queue_list->space_size+=(data_size+queue_list->data_byte_size);
					  //从队列头部取出数据
						uint16_t add=0;
						queue_list->front_add=(queue_list->front_add+queue_list->data_byte_size)% queue_list->list_size;
						while(data_size)
						{
						      //data[add]=queue_list->queue_list[queue_list->front_add];
						      queue_list->front_add=(queue_list->front_add+1)% queue_list->list_size;
								  add++;
								  data_size--;					
						}
						return 1;
			}
			//return 0;
}



//dma入队，只设置队尾地址判断入队是否成功
CCMRAM uint8_t queue_in_dma(uint16_t rear,queue* queue_list)
{
     
     queue_list->rear_add=rear;
     return 1;
		 
}
//获得当前队列存放数据长度
uint16_t use_length;
CCMRAM uint16_t get_queue_use_length(queue*queue_List)
{
      
		use_length=(queue_List->rear_add)>=(queue_List->front_add)?(queue_List->rear_add)-(queue_List->front_add):(queue_List->list_size)-(queue_List->front_add)+(queue_List->rear_add);
    return use_length;
			
}


CCMRAM void queue_out_umpire_system(queue*queue_List)
{
	  uint16_t  length=	get_queue_use_length(queue_List);
   	if((queue_List->list_size)>=length)
		{  //长度不大于列表空间
		  GET_Umpire_DATA(queue_List->queue_list,queue_List->front_add,queue_List->list_size,length);
		}
		else
		{
		  return;
		}
		//出队完成标志
		queue_List->front_add=(queue_List->front_add+length)%queue_List->list_size;
		return;
}

 
 



