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
uint8_t Tx_shijue[34];
uint8_t Tx_shijue_L[8];
uint8_t Tx_shijue_R[8];

extern FDCAN_HandleTypeDef hfdcan3;
extern double yaw12;
extern float gyro[3];
extern receive_vision Rx_vision;
extern int8_t flag;
extern float yaw_angle_L;
extern float yaw_angle_R;
extern float pitch_angle_L;
extern float pitch_angle_R;
extern uint8_t target_shijue;
extern float gyro_yaw_L;
extern float gyro_yaw_R;

void Exchange_Task(void * argument)
{
  for(;;)
  {
		vision_value(); //发送给视觉的数值
		yaw_value(); //云台陀螺仪yaw传到底盘控制5010
    osDelay(1);
  }
}

void vision_value(void)
{
	Tx_vision.header = 0x5A;
	Tx_vision.color = 1;
	Tx_vision.yaw = yaw12;
	Tx_vision.L_yaw = yaw_angle_L + yaw12;
	if(Tx_vision.L_yaw > 180)
	{
		Tx_vision.L_yaw -= 360;
	}
	else if(Tx_vision.L_yaw < -180)
	{
		Tx_vision.L_yaw += 360;
	}
	Tx_vision.L_pitch = pitch_angle_L;
	Tx_vision.R_yaw = yaw_angle_R + yaw12;
	Tx_vision.R_pitch = pitch_angle_R;
	Tx_vision.L_yaw_speed = gyro_yaw_L;
	Tx_vision.R_yaw_speed = gyro_yaw_R;
	Tx_vision.ending = 0xAA;
	memcpy(&Tx_shijue[0], &Tx_vision.header, 1);
	memcpy(&Tx_shijue[1], &Tx_vision.color, 1);
	memcpy(&Tx_shijue[2], &Tx_vision.yaw, 4);
	memcpy(&Tx_shijue[6], &Tx_vision.L_yaw, 4);
	memcpy(&Tx_shijue[10], &Tx_vision.L_pitch, 4);
	memcpy(&Tx_shijue[14], &Tx_vision.R_yaw, 4);
	memcpy(&Tx_shijue[18], &Tx_vision.R_pitch, 4);
	Tx_shijue[22] = target_shijue;
	memcpy(&Tx_shijue[23], &Tx_vision.shoot_speed, 4);
	memcpy(&Tx_shijue[27], &Tx_vision.L_yaw_speed, 4);
	Tx_vision.checksum = Get_CRC16_Check_Sum(Tx_shijue, 31, 0xffff);
	memcpy(&Tx_shijue[31], &Tx_vision.checksum, 2);
	memcpy(&Tx_shijue[33], &Tx_vision.ending, 1);
	CDC_Transmit_HS(Tx_shijue, sizeof(Tx_shijue)); //发给自瞄的值
}

void yaw_value(void)
{
	memcpy(&Tx_yaw12[0], &yaw12, 8); //陀螺仪yaw值
	canx_send_data(&hfdcan3, 0x33, Tx_yaw12, 8);
	osDelay(1);
	
	memcpy(&Tx_gyro[0], &gyro[2], 4); //大yaw角速度
	Tx_gyro[4] = Rx_vision.L_tracking; //yaw的标志位
	Tx_gyro[5] = Rx_vision.R_tracking;
	Tx_gyro[6] = Rx_vision.M_tracking;
	Tx_gyro[7] = flag;
	canx_send_data(&hfdcan3, 0x34, Tx_gyro, 8);
	osDelay(1);
	
	memcpy(&Tx_yaw[0], &Rx_vision.yaw_From_L, 4); //左头识别到时用的，视觉传来yaw的目标值
	memcpy(&Tx_yaw[4], &gyro_yaw_R, 4); //右头6020角速度
	canx_send_data(&hfdcan3, 0x35, Tx_yaw, 8);
	osDelay(1);
	
	Tx_vision.R_yaw = yaw_angle_R + yaw12;
	if(Tx_vision.R_yaw > 180)
	{
		Tx_vision.R_yaw -= 360;
	}
	else if(Tx_vision.R_yaw < -180)
	{
		Tx_vision.R_yaw += 360;
	}
	Tx_vision.R_pitch = pitch_angle_R;
	memcpy(&Tx_shijue_R[0], &Tx_vision.R_yaw, 4);
	memcpy(&Tx_shijue_R[4], &Tx_vision.R_pitch, 4);
	canx_send_data(&hfdcan3, 0x38, Tx_shijue_R, 8);
	osDelay(1);
}
