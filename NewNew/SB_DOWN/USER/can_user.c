#include "can_user.h"
#include "string.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_TxHeaderTypeDef can_tx_message;
uint8_t can_send_data[8];
motor_info motor[8];
motor_5010_info motor_5010;
double yaw12; //云台陀螺仪yaw值
float yaw; //视觉传来的目标yaw值
float yaw_gyro; //云台陀螺仪yaw角速度值
uint8_t L_tracking; //左头是否锁住
uint8_t R_tracking; //右头是否锁住
uint8_t M_tracking; //云台前后摄像头是否锁住
int8_t flag;
float L_yaw;	
float L_pitch;
float R_yaw;	
float R_pitch;

void CAN1_Init()
{
	CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0; 
  can_filter.FilterIdLow  = 0; 
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank  = 14;          
   
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan1);
}

void CAN2_Init()
{
	CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 14;
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0; 
  can_filter.FilterIdLow  = 0; 
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank  = 14;          
   
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
}


/****************************************************CAN1***************************************************/
void can_remote(uint8_t sbus_buf[],uint8_t can_send_id)
{
	CAN_TxHeaderTypeDef can_remote_message;
	uint32_t remote_mail_box = (uint32_t)CAN_TX_MAILBOX0;
	can_remote_message.StdId = can_send_id;
	can_remote_message.IDE = CAN_ID_STD;
	can_remote_message.RTR = CAN_RTR_DATA;
	can_remote_message.DLC = 0x08;
		
	HAL_CAN_AddTxMessage(&hcan1,&can_remote_message,sbus_buf,&remote_mail_box);
}
/*******************************************************************************************************************/


/****************************************************CAN2***************************************************/
void can_cmd_send_3508(int motor1,int motor2,int motor3,int motor4)
{
	uint32_t send_mail_box = (uint32_t)CAN_TX_MAILBOX0;
	can_tx_message.StdId = 0x200;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;
	can_send_data[0] = (motor1>>8)&0xff;
	can_send_data[1] = motor1&0xff;
	can_send_data[2] = (motor2>>8)&0xff;
	can_send_data[3] = motor2&0xff;
	can_send_data[4] = (motor3>>8)&0xff;
	can_send_data[5] = motor3&0xff;
	can_send_data[6] = (motor4>>8)&0xff;
	can_send_data[7] = motor4&0xff;

	HAL_CAN_AddTxMessage(&hcan2,&can_tx_message,can_send_data,&send_mail_box);
}

void can_cmd_send_6020(int motor1,int motor2,int motor3,int motor4)
{
	uint32_t send_mail_box = (uint32_t)CAN_TX_MAILBOX0;
	can_tx_message.StdId = 0x1FF;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;
	can_send_data[0] = (motor1>>8)&0xff;
	can_send_data[1] = motor1&0xff;
	can_send_data[2] = (motor2>>8)&0xff;
	can_send_data[3] = motor2&0xff;
	can_send_data[4] = (motor3>>8)&0xff;
	can_send_data[5] = motor3&0xff;
	can_send_data[6] = (motor4>>8)&0xff;
	can_send_data[7] = motor4&0xff;

	HAL_CAN_AddTxMessage(&hcan2,&can_tx_message,can_send_data,&send_mail_box);
}

void can_cmd_send_5010(uint8_t sbus_buf[])
{
	CAN_TxHeaderTypeDef can_remote_message;
	uint32_t remote_mail_box = (uint32_t)CAN_TX_MAILBOX0;
	can_remote_message.StdId = 0x141;
	can_remote_message.IDE = CAN_ID_STD;
	can_remote_message.RTR = CAN_RTR_DATA;
	can_remote_message.DLC = 0x08;
		
	HAL_CAN_AddTxMessage(&hcan2,&can_remote_message,sbus_buf,&remote_mail_box);
}
/*******************************************************************************************************************/

/*********************************************CAN fifo0*********************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		CAN_RxHeaderTypeDef can_rx_message;
		uint8_t can_recevie_data[8];
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_rx_message,can_recevie_data);
		
		//********************************************************上下板间通信********************************************************//
		if(can_rx_message.StdId == 0x33)
		{
			memcpy(&yaw12, &can_recevie_data[0], 8);
		}
		
		if(can_rx_message.StdId == 0x34)
		{
			memcpy(&yaw_gyro, &can_recevie_data[0], 4);
			L_tracking = can_recevie_data[4];
			R_tracking = can_recevie_data[5];
			M_tracking = can_recevie_data[6];
			flag = can_recevie_data[7];
		}
		
		if(can_rx_message.StdId == 0x35)
		{
			memcpy(&yaw, &can_recevie_data[0], 4);
		}
		
		if(can_rx_message.StdId == 0x38)
		{
			memcpy(&L_yaw, &can_recevie_data[0], 4);
			memcpy(&L_pitch, &can_recevie_data[4], 4);
		}
		
		if(can_rx_message.StdId == 0x39)
		{
			memcpy(&R_yaw, &can_recevie_data[0], 4);
			memcpy(&R_pitch, &can_recevie_data[4], 4);
		}
	}
	
	if(hcan->Instance == CAN2)
	{
		CAN_RxHeaderTypeDef can_rx_message;
		uint8_t can_receive_data[8];
	  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_rx_message,can_receive_data);
	  if((can_rx_message.StdId >= 0x201) && (can_rx_message.StdId <= 0x208))
		{
			uint8_t index = can_rx_message.StdId - 0x201;
			motor[index].angle = ((can_receive_data[0] << 8) | can_receive_data[1]);
			motor[index].speed = ((can_receive_data[2] << 8) | can_receive_data[3]);
			motor[index].tor_current = ((can_receive_data[4] << 8) | can_receive_data[5]);
			motor[index].temperture = can_receive_data[6];
		}
		
		if(can_rx_message.StdId == 0x141)
		{
			if(can_receive_data[0] == 0xA2)
			{
				motor_5010.temperture = can_receive_data[1];
				motor_5010.tor_current = (can_receive_data[2] | (can_receive_data[3] << 8));
				motor_5010.speed = (can_receive_data[4] | (can_receive_data[5] << 8));
				motor_5010.angle = (can_receive_data[6] | (can_receive_data[7] << 8));
			}
		}
	}
}
/*******************************************************************************************************************/
