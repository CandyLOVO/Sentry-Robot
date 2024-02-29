#include "Ins_task.h"
#include "cmsis_os.h"
#include "string.h"

uint8_t Tx[8] = {0x50,0x03,0x00,0x3D,0x00,0x03,0x99,0x86}; //¶ÁÈ¡ÍÓÂÝÒÇ½Ç¶È
uint8_t Tx_key[8] = {0x50,0x06,0x00,0x69,0xB5,0x88,0x22,0xA1}; //½âËø
uint8_t Tx_save[8] = {0x50,0x06,0x00,0x00,0x00,0x00,0x84,0x4B}; //±£´æ

float Roll;
float Pitch;
float Yaw;

extern uint8_t Rx[11];

void InsTask(void const * argument)
{
  for(;;)
  {
		HAL_GPIO_WritePin(GPIOC, DIR_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart2,Tx,sizeof(Tx),0xFF);	
		
		HAL_GPIO_WritePin(GPIOC, DIR_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		osDelay(10);
    osDelay(1);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		Roll = (((short)Rx[3]<<8)|Rx[4])/32768.0*180 - 180;
		Pitch = (((short)Rx[5]<<8)|Rx[6])/32768.0*180;
		Yaw = (((short)Rx[7]<<8)|Rx[8])/32768.0*180 - 180;

		HAL_GPIO_WritePin(GPIOC, DIR_2_Pin|LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_UART_Receive_IT(&huart2,Rx,sizeof(Rx));
	}
}
