#include "user_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_TxHeaderTypeDef can_tx_message;
uint8_t can_send_data[8];
motor_info motor[8];
int n=0;
//void CAN1_Init()
//{
//	CAN_FilterTypeDef  can_filter;

//  can_filter.FilterBank = 0;
//  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;
//  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
//  can_filter.FilterIdHigh = 0; 
//  can_filter.FilterIdLow  = 0; 
//  can_filter.FilterMaskIdHigh = 0;
//  can_filter.FilterMaskIdLow  = 0;
//  can_filter.FilterFIFOAssignment = CAN_RX_FIFO1;
//  can_filter.FilterActivation = ENABLE;
//  can_filter.SlaveStartFilterBank  = 14;          
//   
//  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
//  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO1_MSG_PENDING);
//  HAL_CAN_Start(&hcan1);
//}

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

void can_cmd_send_3508(int motor1,int motor2,int motor3,int motor4) //can1 控制3508四个电机（motor:0-3）
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

	HAL_CAN_AddTxMessage(&hcan1,&can_tx_message,can_send_data,&send_mail_box);
}

void can_cmd_send_6020(int motor1,int motor2,int motor3,int motor4) //can2 控制6020四个电机(motor:4-7)
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

//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) //can1发送3508数据 FIFO1
//{
//	if(hcan->Instance == CAN1){ 
//		CAN_RxHeaderTypeDef can_rx_message;
//		uint8_t can_receive_data[8];
//	  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&can_rx_message,can_receive_data);
//	  if((can_rx_message.StdId >= 0x201) && (can_rx_message.StdId <= 0x204)){
//			uint8_t index = can_rx_message.StdId - 0x201 + 4;
//			motor[index].angle = ((can_receive_data[0] << 8) | can_receive_data[1]);
//			motor[index].speed = ((can_receive_data[2] << 8) | can_receive_data[3]);
//			motor[index].tor_current = ((can_receive_data[4] << 8) | can_receive_data[5]);
//			motor[index].temperture = can_receive_data[6];
//		}
//	}
//}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) //can2发送6020数据 FIFO0
{
	if(hcan->Instance == CAN2)
	{
		CAN_RxHeaderTypeDef can_rx_message;
		uint8_t can_receive_data[8];
	  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_rx_message,can_receive_data);
	  if((can_rx_message.StdId >= 0x205) && (can_rx_message.StdId <= 0x208)){
			uint8_t index = can_rx_message.StdId - 0x201;
			motor[index].angle = ((can_receive_data[0] << 8) | can_receive_data[1]);
			motor[index].speed = ((can_receive_data[2] << 8) | can_receive_data[3]);
			motor[index].tor_current = ((can_receive_data[4] << 8) | can_receive_data[5]);
			motor[index].temperture = can_receive_data[6];
		}
	}

	if(hcan->Instance == CAN1)
	{
		CAN_RxHeaderTypeDef can_rx_message;
		uint8_t can_receive_data[8];
	  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_rx_message,can_receive_data);
	  if((can_rx_message.StdId >= 0x201) && (can_rx_message.StdId <= 0x204)){
			uint8_t index = can_rx_message.StdId - 0x201;
			motor[index].angle = ((can_receive_data[0] << 8) | can_receive_data[1]);
			motor[index].speed = ((can_receive_data[2] << 8) | can_receive_data[3]);
			motor[index].tor_current = ((can_receive_data[4] << 8) | can_receive_data[5]);
			motor[index].temperture = can_receive_data[6];
		}
	}
}

void can_remote(uint8_t sbus_buf[],uint8_t can_send_id) //can通信发送遥控器数据
{
  CAN_TxHeaderTypeDef tx_header;
    
  tx_header.StdId = can_send_id;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf,(uint32_t*)CAN_TX_MAILBOX0);
}