#ifndef __INS_TASK_H__
#define __INS_TASK_H__

#include "main.h"

void INS_Task(void const * argument);
void DRV_USART2_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
