#include "Can_user.h"
#include "remote_control.h"
#include "Yaw_task.h"
#include "Motor.h"
#include "Exchange_task.h"
//	Can 的一些用户撰写的接收函数

int16_t Down_pitch;	//底盘pitch数据

//================================================can1过滤器================================================//
void can_1_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;//标识符寄存器 
  can_filter.FilterIdLow  = 0;//标识符寄存器 
  can_filter.FilterMaskIdHigh = 0;//屏蔽寄存器
  can_filter.FilterMaskIdLow  = 0;       //屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
	HAL_CAN_Start(&hcan1);//启动can，封装在can_user_init()里了
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);		//使能can的FIFO0中断，也封装在can_user_init()里了
}

//================================================can2过滤器================================================//
void can_2_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 14;                       // filter 14
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;//标识符寄存器 
  can_filter.FilterIdLow  = 0;//标识符寄存器 
  can_filter.FilterMaskIdHigh = 0;//屏蔽寄存器
  can_filter.FilterMaskIdLow  = 0;       //屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
	HAL_CAN_Start(&hcan2);//启动can，封装在can_user_init()里了
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);		//使能can的FIFO0中断，也封装在can_user_init()里了
}

//================================================can回调函数(中断)================================================//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//接受中断回调函数
{
  CAN_RxHeaderTypeDef rx_header;
	uint8_t             rx_data[8];
//================================================can1数据================================================//
  if(hcan->Instance == CAN1)
  {
//================================================遥控器数据================================================//
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can1 data
		if(rx_header.StdId == 0x30) //rc_ctrl.rc.ch[0]~[3]
		{
			rc_ctrl.rc.ch[0] = (rx_data[0]<<8) | rx_data[1];
			rc_ctrl.rc.ch[1] = (rx_data[2]<<8) | rx_data[3];
			rc_ctrl.rc.ch[2] = (rx_data[4]<<8) | rx_data[5];
			rc_ctrl.rc.ch[3] = (rx_data[6]<<8) | rx_data[7];
		}
		
		if(rx_header.StdId == 0x31) //rc_ctrl.rc[4]&rc_ctrl.rc.s&rc_ctrl.key
		{
			rc_ctrl.rc.ch[4] = (rx_data[0]<<8) | rx_data[1];
			rc_ctrl.rc.s[0] = rx_data[2];
			rc_ctrl.rc.s[1] = rx_data[3];
			rc_ctrl.key.v = (rx_data[4]<<8) | rx_data[5];
		}
		
		if(rx_header.StdId == 0x32) //rc_ctrl.mouse
		{
			rc_ctrl.mouse.x = (rx_data[0]<<8) | rx_data[1];
			rc_ctrl.mouse.y = (rx_data[2]<<8) | rx_data[3];
			rc_ctrl.mouse.z = (rx_data[4]<<8) | rx_data[5];
			rc_ctrl.mouse.press_l = rx_data[6];
			rc_ctrl.mouse.press_r = rx_data[7];
		}
		
//================================================信息数据================================================//		
		if(rx_header.StdId == 0x54)
		{
			Sentry.Flag_progress = rx_data[0];  //比赛进程
			Sentry.Flag_judge = rx_data[1];  //判断我方是红色方还是蓝色方
			Sentry.Flag_armour = rx_data[2];  //受击打装甲板编号
			Sentry.Time_remain = (rx_data[4]<<8) | rx_data[3];  //比赛剩余时间
			//memcpy(&Sentry.Time_remain,&rx_data[3],2);//与上面等价		
			Sentry.Myself_remain_HP = (rx_data[6]<<8) | rx_data[5];	//本机器人剩余血量			
		}
		
//		if(rx_header.StdId == 0x55)
//		{
//			Sentry.Myself_remain_HP = (rx_data[1]<<8) | rx_data[0];	//本机器人剩余血量
//			Sentry.Myself_17mm_cooling_heat_id1 = (rx_data[3]<<8) | rx_data[2];	//实时枪管1热量
//			Sentry.Myself_17mm_cooling_heat_id2 = (rx_data[5]<<8) | rx_data[4];	//实时枪管2热量	
//		}
		
		if(rx_header.StdId == 0x61)
		{
			Sentry.base_HP = (rx_data[7]<<8) | rx_data[6];
		}
		
		if(rx_header.StdId == 0x63)
		{
			Sentry.event_data = rx_data[6];
		}
		
  }
	
	
//================================================can2数据================================================//
	 if(hcan->Instance == CAN2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
//================================================电机数据接收================================================//		
//接收3508和2006的数据
	if ((rx_header.StdId >= 0x205) && (rx_header.StdId < 0x20B))// 判断标识符，标识符为0x200+ID
  {
		uint8_t index = rx_header.StdId - 0x205;   
    motor_info_can_2[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info_can_2[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info_can_2[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info_can_2[index].temp           =   rx_data[6];
  }
	
//================================================MF9025（注意一下单位）================================================//
int16_t iq;
int16_t speed;
int16_t encoder;	
	if(rx_header.StdId == 0x141)//MF9025电机ID为1
	{
		if(rx_data[0] == 0xA2)	//控制模式下的返回值
		{
			motor_info_can_2[7].torque_current = (rx_data[2] | (rx_data[3] << 8));//先发的低位字节
			motor_info_can_2[7].rotor_speed = (rx_data[4] | (rx_data[5] << 8));
			motor_info_can_2[7].rotor_angle = (rx_data[6] | (rx_data[7] << 8));
		}
		else if(rx_data[0] == 0x90)	//读取编码器（用于初始化）
		{
			motor_info_can_2[7].rotor_angle = (rx_data[2] | (rx_data[3] <<8));//先发的低位字节
		}
	}
  }
}

//================================================遥控器数据板间发送函数================================================//
void can_remote(uint8_t sbus_buf[],uint32_t id)
{
  CAN_TxHeaderTypeDef tx_header;  
  tx_header.StdId = id;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf,(uint32_t*)CAN_TX_MAILBOX0);
}


//================================================can1标准发送函数================================================//
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x1ff);//如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = (v1>>8)&0xff;	//先发高八位		
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

//================================================can2标准发送函数================================================//
void set_motor_voltage_can_2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x1ff);//如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = (v1>>8)&0xff;	//先发高八位		
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
