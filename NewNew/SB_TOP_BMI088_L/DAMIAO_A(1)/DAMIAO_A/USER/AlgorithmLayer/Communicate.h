#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__

#include "main.h"

void DRV_USART5_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
