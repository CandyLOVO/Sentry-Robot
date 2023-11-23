#include "communication.h"
#include "string.h"

extern UART_HandleTypeDef huart1;
extern uint8_t Rx[128];
uint8_t data_length;
uint8_t begin = 0xA5;
uint8_t temp = 0x01; //0x00为视觉数据 0x01为导航数据

DH Tx;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		HAL_UART_DMAStop(&huart1);
		data_length = 128 - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
		if(Rx[0]==begin && Rx[1]==temp){
			memcpy(&Tx.nav_vx,&Rx[47],4);
			memcpy(&Tx.nav_vy,&Rx[51],4);
			memcpy(&Tx.nav_yaw,&Rx[55],4);
//			HAL_UART_Transmit_DMA(&huart6,Rx,data_length);
			HAL_UART_Receive_DMA(&huart1,(uint8_t *)Rx,sizeof(Rx));
		}
	}
}