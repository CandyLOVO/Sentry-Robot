#include "MF_User.h"

//================================================瓴控MF9025电机函数库================================================//
//都是用的can_1

//================================================启动电机================================================//
//para:ID号
void Start_MF_send(int16_t ID)
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x140+ID;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = 0x88;	//先发高八位		
  tx_data[1] = 0x00;
  tx_data[2] = 0x00;	
  tx_data[3] = 0x00;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);
}

//================================================闭环转矩控制================================================//
/*数值范围-2048~ 2048，对应 MF 电机实际转矩电流范围-16.5A~16.5A，对应 MG 电机实际转矩电流范围-33A~33A，
母线电流和电机的实际扭矩因不同电机而异。*/

//para:ID号,电流值的地址
void Current_Control_MF_send(int16_t ID,int16_t *iqControl)
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x140+ID;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = 0xA1;	//先发高八位		
  tx_data[1] = 0x00;
  tx_data[2] = 0x00;	
  tx_data[3] = 0x00;
  tx_data[4] = *(uint8_t *)(&iqControl);
  tx_data[5] =  *((uint8_t *)(&iqControl)+1);
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);
}

//================================================MF电机闭环转矩控制电流限制================================================//
//输入：电流值
//输出：限制后的电流值
int16_t Current_Limit_MF(int16_t current)
{
	int16_t iqControl;
	if(current > 2048)
		iqControl = 2048;
	else if(current < -2048)
		iqControl = -2048;
	else
		iqControl = current;
	return iqControl;
}

//示例接收函数（注意单位）
//HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//	if(rx_header.StdId == 0x141)//MF9025电机ID为1
//	{
//		if(rx_data[0] == 0xA1)	//控制模式下的返回值
//		{
//			motor_info[0].torque_current = (rx_data[2] | (rx_data[3] << 8));//先发的低位字节
//			motor_info[0].rotor_speed = (rx_data[4] | (rx_data[5] << 8));
//			motor_info[0].rotor_angle = (rx_data[6] | (rx_data[7] << 8));
//		}
//	}