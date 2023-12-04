#include "Can_user.h"
#include "remote_control.h"
#include "Yaw_task.h"

//	Can 的一些用户撰写的接收函数
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
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
		if(rx_header.StdId==0x33)//双C板传递遥控器信号的接口标识符
		{
			rc_ctrl.rc.ch[0] = (rx_data[0] | (rx_data[1] << 8)) & 0x07ff;        //!< Channel 0  中值为1024，最大值1684，最小值364，波动范围：660
			rc_ctrl.rc.ch[1] = (((rx_data[1] >> 3)&0xff) | (rx_data[2] << 5)) & 0x07ff; //!< Channel 1
			rc_ctrl.rc.ch[2] = (((rx_data[2] >> 6)&0xff) | (rx_data[3] << 2) |          //!< Channel 2
                         (rx_data[4] << 10)) &0x07ff;
			rc_ctrl.rc.ch[3] = (((rx_data[4] >> 1)&0xff) | (rx_data[5] << 7)) & 0x07ff; //!< Channel 3
			rc_ctrl.rc.s[0] = ((rx_data[5] >> 4) & 0x0003);                  //这是右
			rc_ctrl.rc.s[1] = ((rx_data[5] >> 4) & 0x000C) >> 2;    		//这才是左
			rc_ctrl.mouse.x = rx_data[6] | (rx_data[7] << 8);                    //!< Mouse X axis
			}
		if(rx_header.StdId==0x34)//双C板传递遥控器信号的接口标识符
		{
			rc_ctrl.mouse.y = rx_data[0] | (rx_data[1] << 8);                    //!< Mouse Y axis
			rc_ctrl.mouse.z = rx_data[2] | (rx_data[3] << 8);                  //!< Mouse Z axis
			rc_ctrl.mouse.press_l = rx_data[4];                                  //!< Mouse Left Is Press ?
			rc_ctrl.mouse.press_r = rx_data[5];                                  //!< Mouse Right Is Press ?
			rc_ctrl.key.v = rx_data[6] | (rx_data[7] << 8); 
		}
		if(rx_header.StdId==0x35)//双C板传递遥控器信号的接口标识符
		{
    rc_ctrl.rc.ch[4] = rx_data[0] | (rx_data[1] << 8);                 
			
//================================================底盘数据================================================//
		//接收底盘旋转量(用来修正yaw轴，作为前馈控制)
			Rotate_w = (rx_data[3] << 8) | rx_data[4];
			
		//接收底盘imu的Pitch数据(用来检测上下坡限位)
			Down_pitch = (rx_data[5] << 8) | rx_data[6];
			
		}

//================================================光电门矫正yaw================================================//		
		//YAW校正接收(光电门，未使用)
		if(rx_header.StdId==0x66)
		{
			if(rx_data[0] == 0xff)
			{
				Update_yaw_flag = 1;
				target_yaw = 0;//回正锁云台的值
			}
		}
	
//================================================裁判系统================================================//		
		//比赛进程标识符(裁判系统数据)
		if(rx_header.StdId==0x10)
		{
			Sentry.Flag_progress = rx_data[0];
			Sentry.Flag_judge = rx_data[1];
		}
		
		
//================================================电机数据接收================================================//		
//接收3508和2006的数据
	if ((rx_header.StdId >= 0x201) && (rx_header.StdId < 0x208))// 判断标识符，标识符为0x200+ID
  {
    uint8_t index = rx_header.StdId - 0x201;                  // get motor index by can_id
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
  }
	
	//对Yaw(6020电机)进行单独的处理，其6020电机ID为6，对其单独赋值
	else if(rx_header.StdId == 0x20A)
	{
		motor_info[6].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[6].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[6].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[6].temp           =   rx_data[6];
	}

	
  }
	
	
//================================================can2数据================================================//
	 if(hcan->Instance == CAN2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
//================================================电机数据接收================================================//	
//底盘4个3508麦轮电机数据
	if ((rx_header.StdId >= 0x201) && (rx_header.StdId <  0x205)) 
  {
    uint8_t index = rx_header.StdId - 0x201;
    motor_info_can_2[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info_can_2[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info_can_2[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info_can_2[index].temp           =   rx_data[6];

  }
	
//6020数据，这里注意6020的id需要大于4，不然会覆盖3508底盘电机的值
	  if ((rx_header.StdId >= 0x205) && (rx_header.StdId <  0x20F))
  {
    uint8_t index = rx_header.StdId - 0x205;
    motor_info_can_2[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info_can_2[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info_can_2[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info_can_2[index].temp           =   rx_data[6];

  }
  }
}

//================================================遥控器数据板间发送函数================================================//
//注：有用来给下c板发yaw的信息
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
