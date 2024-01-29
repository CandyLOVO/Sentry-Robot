#include "user_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

motor_info motor[4];
motor_info motor_9025;

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

//CAN通信控制6020电机 —— CAN2
void can_send_6020(int motor1,int motor2,int motor3,int motor4)
{
	CAN_TxHeaderTypeDef can_tx_messgae;
	uint8_t can_send_data[8];
	uint32_t send_mail_box;
	can_tx_messgae.StdId = 0x1ff;
	can_tx_messgae.IDE = CAN_ID_STD;
	can_tx_messgae.RTR = CAN_RTR_DATA;
	can_tx_messgae.DLC = 0x08;
	can_send_data[0] = (motor1>>8)&0xff;
	can_send_data[1] = motor1&0xff;
	can_send_data[2] = (motor2>>8)&0xff;
	can_send_data[3] = motor2&0xff;
	can_send_data[4] = (motor3>>8)&0xff;
	can_send_data[5] = motor3&0xff;
	can_send_data[6] = (motor4>>8)&0xff;
	can_send_data[7] = motor4&0xff;
	
	HAL_CAN_AddTxMessage(&hcan2,&can_tx_messgae,can_send_data,&send_mail_box);
}

//CAN通信控制9025电机速度 —— CAN2
void can_send_9025(int32_t speedControl)
{
	CAN_TxHeaderTypeDef can_tx_messgae;
	uint8_t can_send_data[8];
	uint32_t send_mail_box;
	can_tx_messgae.StdId = 0x141;
	can_tx_messgae.IDE = CAN_ID_STD;
	can_tx_messgae.RTR = CAN_RTR_DATA;
	can_tx_messgae.DLC = 0x08;
	can_send_data[0] = 0xA2;
	can_send_data[1] = 0;
	can_send_data[2] = 0;
	can_send_data[3] = 0;
	can_send_data[4] = *(uint8_t*)(&speedControl);
	can_send_data[5] = *((uint8_t*)(&speedControl)+1);
	can_send_data[6] = *((uint8_t*)(&speedControl)+2);
	can_send_data[7] = *((uint8_t*)(&speedControl)+3);
	
	HAL_CAN_AddTxMessage(&hcan2,&can_tx_messgae,can_send_data,&send_mail_box);
}

//CAN通信接收电机信息
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN2)
	{
		CAN_RxHeaderTypeDef can_rx_message;
		uint8_t can_receive_data[8];
	  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_rx_message,can_receive_data);
		
		//接收四个6020返回值
	  if((can_rx_message.StdId >= 0x205) && (can_rx_message.StdId <= 0x208)){
			uint8_t index = can_rx_message.StdId - 0x205;
			motor[index].angle = ((can_receive_data[0] << 8) | can_receive_data[1]);
			motor[index].speed = ((can_receive_data[2] << 8) | can_receive_data[3]);
			motor[index].tor_current = ((can_receive_data[4] << 8) | can_receive_data[5]);
			motor[index].temperture = can_receive_data[6];
		}
		
		//接收9025返回值
		if(can_rx_message.StdId == 0x141 && can_receive_data[0] == 0xA2){
			motor_9025.temperture = can_receive_data[1];
			motor_9025.tor_current = ((can_receive_data[2] << 8) | can_receive_data[3]);
			motor_9025.speed = ((can_receive_data[4] << 8) | can_receive_data[5]);
			motor_9025.angle = ((can_receive_data[6] << 8) | can_receive_data[7]); //编码值位置
		}
	}
}

//遥控器CAN通信发送到上/下C板 —— CAN1
void can_remote(uint8_t sbus_buf[],uint8_t can_send_id)
{
	CAN_TxHeaderTypeDef can_remote_message;
	uint32_t remote_mail_box = (uint32_t)CAN_TX_MAILBOX1;
	can_remote_message.StdId = can_send_id;
	can_remote_message.IDE = CAN_ID_STD;
	can_remote_message.RTR = CAN_RTR_DATA;
	can_remote_message.DLC = 0x08;
	
	HAL_CAN_AddTxMessage(&hcan1,&can_remote_message,sbus_buf,&remote_mail_box);
}
