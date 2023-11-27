#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "main.h"

typedef struct DaoHang{
	float nav_vx;
	float nav_vy;
	float nav_yaw;
}DH;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#endif