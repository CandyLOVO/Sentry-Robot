#include "Communicate.h"
#include "string.h"
#include "Exchange_task.h"

extern uint8_t Rx_R[128];
extern UART_HandleTypeDef huart5;
extern Vision_receive_t vision_receive;

void DRV_USART5_IRQHandler(UART_HandleTypeDef *huart) //右头与视觉通信 //在stm32f4xx_it.c文件USART1_IRQHandler调用
{
	if(huart->Instance == UART5)
	{
		if(RESET != __HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE)){
			__HAL_UART_CLEAR_IDLEFLAG(&huart5);
			HAL_UART_DMAStop(&huart5);
			if(Rx_R[0]==0xA5){
				memcpy(&vision_receive.R_tracking,&Rx_R[1],1);
				memcpy(&vision_receive.R_shoot,&Rx_R[2],1);
				memcpy(&vision_receive.R_chase_yaw,&Rx_R[3],4);
				memcpy(&vision_receive.R_chase_pitch,&Rx_R[7],4);
				memcpy(&vision_receive.R_distance,&Rx_R[11],4);
			}
			HAL_UART_Receive_DMA(&huart5,(uint8_t *)Rx_R,sizeof(Rx_R));
		}
	}
}
