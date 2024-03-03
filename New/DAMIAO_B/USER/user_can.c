#include "user_can.h"
#include "rc_potocal.h"
#include "string.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern RC_ctrl_t rc_ctrl;
motor_info motor_m3508[6];/*摩擦轮电机 id 1~4*/
motor_info motor_m2006[8];/*拨盘电机 id 5~6*/
Shooter_t Shooter_L;
Shooter_t Shooter_R;
uint8_t Flag_mode;
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

//CAN2通信控制摩擦轮 c620电调 电机id 1234
void can_send_mocalun(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
	CAN_TxHeaderTypeDef can_tx_messgae;
	uint8_t can_send_data[8];
	uint32_t send_mail_box;
	can_tx_messgae.StdId = 0x200;
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

//CAN2通信控制拨盘 c610电调 电机id 5，6
void can_send_bopan(int16_t motor1,int16_t motor2)
{
	CAN_TxHeaderTypeDef can_tx_messgae;
	uint8_t can_send_data[8];
	uint32_t send_mail_box;
	can_tx_messgae.StdId = 0x1FF;
	can_tx_messgae.IDE = CAN_ID_STD;
	can_tx_messgae.RTR = CAN_RTR_DATA;
	can_tx_messgae.DLC = 0x08;
	can_send_data[0] = (motor1>>8)&0xff;
	can_send_data[1] = motor1&0xff;
	can_send_data[2] = (motor2>>8)&0xff;
	can_send_data[3] = motor2&0xff;
	can_send_data[4] = 0;
	can_send_data[5] = 0;
	can_send_data[6] = 0;
	can_send_data[7] = 0;
	
	HAL_CAN_AddTxMessage(&hcan2,&can_tx_messgae,can_send_data,&send_mail_box);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{ //CAN1通信接受信息 
	if(hcan->Instance == CAN1)
	{CAN_RxHeaderTypeDef can_rx_message;
		uint8_t can_receive_data[8];
	  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_rx_message,can_receive_data);
		
		//接收发射标志
	  if(can_rx_message.StdId == 0x53)
		{Flag_mode = can_receive_data[0];              //发射模式
		 Shooter_L.Fire_Flag = can_receive_data[1];   //视觉目标识别标识left
			Shooter_R.Fire_Flag = can_receive_data[2];   //视觉目标识别表示right
		}
	
		if(can_rx_message.StdId == 0x61)//左枪管数据接收
   {
     Shooter_L.speed_limit = (can_receive_data[1]<<8) | can_receive_data[0];        //射速上限
     Shooter_L.heat_limit = (can_receive_data[3]<<8) | can_receive_data[2];        //热量上限
     Shooter_L.cooling_rate = (can_receive_data[5]<<8) | can_receive_data[4];     //热量每秒冷却值      
	 }
	 if(can_rx_message.StdId == 0x62)
	 {
		 memcpy(&Shooter_L.shoot_speed,&can_receive_data[0],4);                  //实时射速（m/s）
		 Shooter_L.shooter_heat=(can_receive_data[5]<<8) | can_receive_data[4]; //实时枪管热量
		 Shooter_L.shoot_rate=can_receive_data[6];                             //实时射频（hz）
	 }
	 		if(can_rx_message.StdId == 0x63)//右枪管数据接收
   {
     Shooter_R.speed_limit = (can_receive_data[1]<<8) | can_receive_data[0];        //射速上限
     Shooter_R.heat_limit = (can_receive_data[3]<<8) | can_receive_data[2];        //热量上限
     Shooter_R.cooling_rate = (can_receive_data[5]<<8) | can_receive_data[4];     //热量每秒冷却值      
	 }
	 if(can_rx_message.StdId == 0x64)
	 {
		 memcpy(&Shooter_R.shoot_speed,&can_receive_data[0],4);                  //实时射速（m/s）
		 Shooter_R.shooter_heat=(can_receive_data[5]<<8) | can_receive_data[4]; //实时枪管热量
		 Shooter_R.shoot_rate=can_receive_data[6];                             //实时射频（hz）
	 }
	 
	 
	 if(can_rx_message.StdId == 0x30)
     {
     rc_ctrl.rc.ch[0] = (can_receive_data[0]<<8) | can_receive_data[1];
     rc_ctrl.rc.ch[1] = (can_receive_data[2]<<8) | can_receive_data[3];
     rc_ctrl.rc.ch[2] = (can_receive_data[4]<<8) | can_receive_data[5];
     }
                
     if(can_rx_message.StdId == 0x31)
     {
     rc_ctrl.rc.ch[4] = (can_receive_data[0]<<8) | can_receive_data[1];
     rc_ctrl.rc.s[0] = can_receive_data[2];
     rc_ctrl.rc.s[1] = can_receive_data[3];
     rc_ctrl.key.v = (can_receive_data[4]<<8) | can_receive_data[5];
     }
                
     if(can_rx_message.StdId == 0x32)
     {
     rc_ctrl.mouse.x = (can_receive_data[0]<<8) | can_receive_data[1];
     rc_ctrl.mouse.y = (can_receive_data[2]<<8) | can_receive_data[3];
     rc_ctrl.mouse.z = (can_receive_data[4]<<8) | can_receive_data[5];
     rc_ctrl.mouse.press_l = can_receive_data[6];
     rc_ctrl.mouse.press_r = can_receive_data[7];
     }
		
	
	}
	//CAN2通信接收电机信息
	if(hcan->Instance == CAN2)
	{
		CAN_RxHeaderTypeDef can_rx_message;
		uint8_t can_receive_data[8];
	  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_rx_message,can_receive_data);
		
		/*接收四个摩擦轮3508返回值*/
	  if((can_rx_message.StdId >= 0x201) && (can_rx_message.StdId <= 0x204)){
			uint8_t index = can_rx_message.StdId - 0x200;
			motor_m3508[index].angle = ((can_receive_data[0] << 8) | can_receive_data[1]);
			motor_m3508[index].speed = ((can_receive_data[2] << 8) | can_receive_data[3]);
			motor_m3508[index].tor_current = ((can_receive_data[4] << 8) | can_receive_data[5]);
			motor_m3508[index].temperture = can_receive_data[6];
		}
		
		//接收拨盘m2006返回值
		if(can_rx_message.StdId >= 0x205 && can_rx_message.StdId <= 0x206){
			uint8_t index = can_rx_message.StdId - 0x200;
			motor_m2006[index].angle = ((can_receive_data[0] << 8) | can_receive_data[1]);
			motor_m2006[index].speed = ((can_receive_data[2] << 8) | can_receive_data[3]);
			motor_m2006[index].tor_current = ((can_receive_data[4] << 8) | can_receive_data[5]);
		}
	}
}

