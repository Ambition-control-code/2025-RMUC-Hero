#include "User_com_all.h"

uint8_t   USART1_RX_BUFF[USART1_RX_BUFF_LENGTH];
uint8_t   USART1_TX_BUFF[USART1_TX_BUFF_LENGTH];

uint8_t   USART3_RX_BUFF[USART3_RX_BUFF_LENGTH];
uint8_t   USART3_TX_BUFF[USART3_TX_BUFF_LENGTH];

uint16_t IDLE_datalength=0;
void USART_SET_INIT(void)
{
	  queue_rx_init();
	  SUPER_Um_State_INIT();
	
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                          //使能空闲中断
    HAL_UART_Receive_DMA(&huart1,USART1_RX_BUFF,USART1_RX_BUFF_LENGTH);   //使能串口接收DMA
	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                          //使能空闲中断
      HAL_UART_Receive_DMA(&huart3,USART3_RX_BUFF,USART3_RX_BUFF_LENGTH);   //使能串口接收DMA
}

CCMRAM void USART1_IRQ_IDLE_CALLBACK(void)
{
		/* USER CODE BEGIN USART1_IRQn 0 */
	  HAL_UART_DMAStop(&huart1); //  停止DMA传输				
		if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)==SET)   //判断是否是空闲中断
		{
			  __HAL_UART_CLEAR_IDLEFLAG(&huart1);                                 //清楚空闲中断标志
				
			  //当空闲中断发生时
				//获取数据长度
			IDLE_datalength=USART1_RX_BUFF_LENGTH-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		
		  if(IDLE_datalength==RC_FRAME_LENGTH)
			{
			  SBUS_TO_RC(USART1_RX_BUFF,&Remote_Ctrl);
			}
		}			
		//receive_num=MAX_BUF-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		HAL_UART_Receive_DMA(&huart1,USART1_RX_BUFF,USART1_RX_BUFF_LENGTH);   //使能串口接收DMA
}

CCMRAM void USART1_XferHalfCpltCallback2(DMA_HandleTypeDef *hdma)
{
   	//a=__HAL_DMA_GET_COUNTER(hdma);
    if(__HAL_DMA_GET_FLAG(hdma,DMA_FLAG_HT5) != RESET)
		//半中断，意味着前一半的数据可以使用
    {
//         queue_in_dma(USART1_RX_BUFF_LENGTH/2-1,&USART1_RX_queue);  
		}
		else if(__HAL_DMA_GET_FLAG(hdma,DMA_FLAG_TC5) != RESET)
		//全中断，意味着后一半数据可用
		{
//		     queue_in_dma(USART1_RX_BUFF_LENGTH-1,&USART1_RX_queue);   
		}
}

CCMRAM void USART3_IRQ_IDLE_CALLBACK(void)
{
		/* USER CODE BEGIN USART1_IRQn 0 */
		if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)==SET)   //判断是否是空闲中断
		{
			  __HAL_UART_CLEAR_IDLEFLAG(&huart3);                                 //清楚空闲中断标志			
			  //当空闲中断发生时
				//获取数据长度
				IDLE_datalength=USART3_RX_BUFF_LENGTH-__HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		    //空闲中断位置就是队尾
        queue_in_dma(IDLE_datalength,&USART3_RX_queue);
		}
			  //HAL_UART_DMAStop(&huart1); //  停止DMA传输				
			  //receive_num=MAX_BUF-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
			 	//HAL_UART_Receive_DMA(&huart1,u1_rec_buf,MAX_BUF);//重新打开DMA接收	
}


CCMRAM void USART3_XferHalfCpltCallback2(DMA_HandleTypeDef *hdma)
{
   	//a=__HAL_DMA_GET_COUNTER(hdma);
    if(__HAL_DMA_GET_FLAG(hdma,DMA_FLAG_HT5) != RESET)
		//半中断，意味着前一半的数据可以使用
    {
         queue_in_dma(USART3_RX_BUFF_LENGTH/2-1,&USART3_RX_queue);  
		}
		else if(__HAL_DMA_GET_FLAG(hdma,DMA_FLAG_TC5) != RESET)
		//全中断，意味着后一半数据可用
		{
   			 queue_in_dma(USART3_RX_BUFF_LENGTH-1,&USART3_RX_queue);   
		}

}

CCMRAM void USART3_TX_INT(void)
{
	if(__HAL_DMA_GET_FLAG(&hdma_usart3_tx,DMA_FLAG_TC1) != RESET)
	{
		  DRAW_DMA_OK_STATE=1;
	}
}


