#ifndef __USART_header
#define __USART_header

#include "stm32f4xx_hal.h"

#define RX_BUF_SIZE     10

extern UART_HandleTypeDef *USART_Used;
extern uint8_t RX_Buffer[RX_BUF_SIZE];
extern HAL_StatusTypeDef RX_Task(void);

#endif /* __USART_header */