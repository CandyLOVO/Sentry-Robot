#include "Exchange_Task.h"
#include "cmsis_os.h"
#include "uart_user.h"
#include "CRC.h"
#include "string.h"

uint8_t Tx[13];
uint16_t checksum;

extern UART_HandleTypeDef huart1;
extern uint8_t Rx[128];

void Exchange_Task(void const * argument)
{
  for(;;)
  {
		RS485_Trans();
    osDelay(1);
  }
}

void RS485_Trans(void)
{
	Tx[0] = 0x5A;
//	for(int i=0;i<200;i++)
//	{
		Tx[1] = 0;
		Tx[2] = 0;
		Tx[3] = 0x04;
		Tx[4] = 0x08;
		Tx[5] = 0xA0;
		Tx[6] = 0x56;
		Tx[7] = 0x04;
		Tx[8] = 0x01;
		Tx[9] = 0x34;
//	}
	checksum = Get_CRC16_Check_Sum(Tx,10,0xffff);
	memcpy(&Tx[10],&checksum,2);
	Tx[12] = 0xAA;
	HAL_GPIO_WritePin(DIR_2_GPIO_Port,DIR_2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_1_GPIO_Port,DIR_1_Pin,GPIO_PIN_SET);
  HAL_UART_Transmit_DMA(&huart1,Tx,sizeof(Tx));
	osDelay(5);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
    	HAL_GPIO_WritePin(DIR_2_GPIO_Port,DIR_2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIR_1_GPIO_Port,DIR_1_Pin,GPIO_PIN_RESET);
    }
}

