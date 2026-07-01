#include "referee_system.h"

void USART5_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART5->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart5);

        static uint16_t this_time_rx_len = 0;

        if ((huart5.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart5.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart5.hdmarx);
            __HAL_DMA_SET_COUNTER(huart5.hdmarx, USART_RX_BUF_LENGHT);
            huart5.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart5.hdmarx);
        }
        else
        {
            __HAL_DMA_DISABLE(huart5.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart5.hdmarx, USART_RX_BUF_LENGHT);
            huart5.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart5.hdmarx);
        }
    }
}


