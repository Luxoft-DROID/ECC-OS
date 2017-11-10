#include "USART.h"

UART_HandleTypeDef *USART_Used;

uint8_t RX_Buffer[RX_BUF_SIZE];

HAL_StatusTypeDef RX_Task(void)
{
  static char RX_counter = 0;
  HAL_StatusTypeDef Returned_Stat = HAL_OK;
  
  Returned_Stat = HAL_UART_Receive(USART_Used, (uint8_t *)&RX_Buffer[RX_counter], 1, 20); 
  
  if(HAL_TIMEOUT == Returned_Stat)
    {
      if(RX_BUF_SIZE == RX_counter)
        {
          Returned_Stat = HAL_OK;
        }
       else
        {
          Returned_Stat = HAL_TIMEOUT;
        }
      RX_counter = 0x00;
    }
    else
    {
      if( RX_BUF_SIZE > RX_counter)
        {
          RX_counter++;
          Returned_Stat = HAL_TIMEOUT;
        }
       else
        {
          RX_counter = 0x00;
          Returned_Stat = HAL_TIMEOUT;
        }
    }
  return (Returned_Stat);
}