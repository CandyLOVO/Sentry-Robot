#include "Exchange_task.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "can_user.h"
#include "imu_temp_ctrl.h"
#include "string.h"
#include "CRC.h"
#include "usbd_cdc_if.h"

transmit_vision Tx_vision;
uint8_t Tx_yaw12[8];
uint8_t Tx_yaw[8];
uint8_t Tx_gyro[8];

extern FDCAN_HandleTypeDef hfdcan3;
extern double yaw12;
extern float gyro[3];
extern receive_vision Rx_vision;

void Exchange_Task(void const * argument)
{
  for(;;)
  {
		vision_value();
		yaw_value(); //云台陀螺仪yaw传到底盘控制5010
    osDelay(1);
  }
}

void vision_value(void)
{
	
//	CDC_Transmit_HS(Tx_vision, sizeof(Tx_vision)); //发给自瞄的值
}

void yaw_value(void)
{
	memcpy(&Tx_yaw12[0], &yaw12, 8); //陀螺仪yaw值
	canx_send_data(&hfdcan3, 0x33, Tx_yaw12, 8);
	
	memcpy(&Tx_gyro[0], &gyro[2], 4); //大yaw角速度
	Tx_gyro[4] = Rx_vision.L_tracking; //yaw的标志位
	Tx_gyro[5] = Rx_vision.R_tracking;
	Tx_gyro[6] = Rx_vision.M_tracking;
	canx_send_data(&hfdcan3, 0x34, Tx_gyro, 8);
	
	memcpy(&Tx_yaw[0], &Rx_vision.yaw, 4); //视觉传来yaw的目标值
	canx_send_data(&hfdcan3, 0x35, Tx_yaw, 8);
}
