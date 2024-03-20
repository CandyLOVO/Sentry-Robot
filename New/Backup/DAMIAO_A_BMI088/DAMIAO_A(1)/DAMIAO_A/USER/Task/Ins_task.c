#include "Ins_task.h"
#include "cmsis_os.h"
#include "string.h"
#include "imu_temp_control_task.h"

//uint8_t Tx[8] = {0x50,0x03,0x00,0x30,0x00,0x30,0x48,0x50};
uint8_t Tx[8] = {0x50,0x03,0x00,0x3D,0x00,0x03,0x99,0x86}; //读取陀螺仪角度
uint8_t Tx_z0[8] = {0x50,0x06,0x00,0x01,0x00,0x04,0xD4,0x48}; //z轴数据归零
uint8_t Tx_key[8] = {0x50,0x06,0x00,0x69,0xB5,0x88,0x22,0xA1}; //解锁
uint8_t Tx_save[8] = {0x50,0x06,0x00,0x00,0x00,0x00,0x84,0x4B}; //保存

int16_t GX;
int16_t GY;
int16_t GZ;
float Roll;
float Pitch;
float Yaw;

extern uint8_t Rx[128];
extern float Yaw_middle_c;	//一级云台yaw(只有绝对坐标) 9025转化为0~+-180后的编码值

void InsTask(void const * argument)
{
  for(;;)
  {
//		HAL_GPIO_WritePin(GPIOC, DIR_2_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
//		HAL_UART_Transmit(&huart2,Tx,sizeof(Tx),0xFF);	
//		
//		HAL_GPIO_WritePin(GPIOC, DIR_2_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
//		osDelay(5);
//		osDelay(1);

		imu_temp_control_task();
//		    osDelay(1);
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

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == USART2)
//	{
//		HAL_UART_DMAStop(&huart2);
//		if(Rx[0]==0x50 && Rx[1]==0x03)
//		{
////			GX = (((short)Rx[17]<<8)|Rx[18])/32768.0*2000; //度/s
////			GY = (((short)Rx[19]<<8)|Rx[20])/32768.0*2000;
////			GZ = (((short)Rx[21]<<8)|Rx[22])/32768.0*2000;
////			
////			GX = GX*60/360.0; //Roll RPM
////			GY = GY*60/360.0; //Pitch
////			GZ = GZ*60/360.0; //Yaw
////			
////			Roll = (((short)Rx[29]<<8)|Rx[30])/32768.0*180; //角度制 0~360
////			Pitch = (((short)Rx[31]<<8)|Rx[32])/32768.0*180;
////			Yaw = (((short)Rx[33]<<8)|Rx[34])/32768.0*180;
//			
//			Roll = (((short)Rx[3]<<8)|Rx[4])/32768.0*180; //角度制 0~360
//			Pitch = (((short)Rx[5]<<8)|Rx[6])/32768.0*180;
//			Yaw = (((short)Rx[7]<<8)|Rx[8])/32768.0*180;
//			if(Yaw>180)
//			{
//				Yaw -= 360;
//			}
//			Yaw_middle_c = Yaw;
//		}
//		HAL_UART_Receive_DMA(&huart2,(uint8_t *)Rx,sizeof(Rx));
//	}
//}
