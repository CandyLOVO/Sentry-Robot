#include "Exchange_Task.h"
#include "cmsis_os.h"
#include "uart_user.h"
#include "CRC.h"
#include "string.h"

uint8_t Tx[13];
uint16_t checksum;
uint8_t Rx_flag = 0;

extern UART_HandleTypeDef huart1;
extern uint8_t Rx[128];
extern uint32_t length; //DMA中未传输的数据个数
extern int count;

void Exchange_Task(void const * argument)
{
  for(;;)
  {
		if(Rx_flag==0) //程序初始化标志位
		{
			RS485_Trans();
			Rx_flag = 1;
		}
		else if(Rx_flag == 2) //收到一帧数据后
		{
			RS485_Trans();
			Rx_flag = 1;
		}
		else
		{
			count++; //收到一帧数据后置零
		}
		
		if(count>=30) //如果有30ms没有收到导航数据
		{
			RS485_Trans();
			count = 0;
		}
		
    osDelay(1);
  }
}

void RS485_Trans(void)
{
	Tx[0] = 0x5A;
	Tx[1] = 0;
	Tx[2] = 0;
	Tx[3] = 0x04;
	Tx[4] = 0x08;
	Tx[5] = 0xA0;
	Tx[6] = 0x56;
	Tx[7] = 0x04;
	Tx[8] = 0x01;
	Tx[9] = 0x34;
	checksum = Get_CRC16_Check_Sum(Tx,10,0xffff);
	memcpy(&Tx[10],&checksum,2);
	Tx[12] = 0xAA;
	
	HAL_GPIO_WritePin(DIR_2_GPIO_Port,DIR_2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_1_GPIO_Port,DIR_1_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit_IT(&huart1,Tx,sizeof(Tx));
	osDelay(10);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
			HAL_GPIO_WritePin(DIR_2_GPIO_Port,DIR_2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIR_1_GPIO_Port,DIR_1_Pin,GPIO_PIN_RESET);
    }
}
