#ifndef __INS_TASK_H__
#define __INS_TASK_H__

#include "main.h"

extern UART_HandleTypeDef huart2;

void DRV_USART2_IRQHandler(UART_HandleTypeDef *huart);
void InsTask(void const * argument);

#endif
