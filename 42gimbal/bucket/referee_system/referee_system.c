#include "referee_system.h"
#include "stm32f4xx.h"

extern UART_HandleTypeDef huart5;


void USART5_IRQHandler(void)
{
    static volatile uint8_t res;
    if(huart5.Instance->SR& UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart5);

        static uint16_t this_time_rx_len = 0;

        if ((huart5.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart5.hdmarx);
            this_time_rx_len = 512 - __HAL_DMA_GET_COUNTER(huart5.hdmarx);
            __HAL_DMA_SET_COUNTER(huart5.hdmarx, 512);
            huart5.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart5.hdmarx);
        }
        else
        {
            __HAL_DMA_DISABLE(huart5.hdmarx);
            this_time_rx_len=512-__HAL_DMA_GET_COUNTER(huart5.hdmarx);
            __HAL_DMA_SET_COUNTER(huart5.hdmarx, 512);
            huart5.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart5.hdmarx);
        }
    }
}


