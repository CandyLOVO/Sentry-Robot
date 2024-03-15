#include "INS_Task.h"
#include "cmsis_os.h"
#include "string.h"

uint8_t Tx[8] = {0x50,0x03,0x00,0x30,0x00,0x30,0x48,0x50};
//uint8_t Tx[8] = {0x50,0x03,0x00,0x3D,0x00,0x03,0x99,0x86}; //��ȡ�����ǽǶ�
uint8_t Tx_z0[8] = {0x50,0x06,0x00,0x01,0x00,0x04,0xD4,0x48}; //z�����ݹ���
uint8_t Tx_key[8] = {0x50,0x06,0x00,0x69,0xB5,0x88,0x22,0xA1}; //����
uint8_t Tx_save[8] = {0x50,0x06,0x00,0x00,0x00,0x00,0x84,0x4B}; //����

float GX;
float GY;
float GZ;
float Roll;
float Pitch;
float Yaw;

extern UART_HandleTypeDef huart2;
extern uint8_t Rx[101];
extern float Yaw_middle_c;	//һ����̨yaw(ֻ�о�������) 9025ת��Ϊ0~+-180��ı���ֵ

void INS_Task(void const * argument)
{
  for(;;)
  {
		HAL_GPIO_WritePin(GPIOC, DIR_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart2,Tx,sizeof(Tx),0xFF);	
		
		HAL_GPIO_WritePin(GPIOC, DIR_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		osDelay(5);
    osDelay(1);
    osDelay(1);
  }
}

void DRV_USART2_IRQHandler(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		if(RESET != __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)){
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);
			HAL_UART_RxCpltCallback(huart);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		HAL_UART_DMAStop(&huart2);
		if(Rx[0]==0x50 && Rx[1]==0x03)
		{
			GX = (((short)Rx[11]<<8)|Rx[12])/32768*2000;
			GY = (((short)Rx[13]<<8)|Rx[14])/32768*2000;
			GZ = (((short)Rx[15]<<8)|Rx[16])/32768*2000;
			
			Roll = (((short)Rx[29]<<8)|Rx[30])/32768.0*180; //�Ƕ��� 0~360
			Pitch = (((short)Rx[31]<<8)|Rx[32])/32768.0*180;
			Yaw = (((short)Rx[33]<<8)|Rx[34])/32768.0*180;
		}
		HAL_UART_Receive_DMA(&huart2,(uint8_t *)Rx,sizeof(Rx));
	}
}

