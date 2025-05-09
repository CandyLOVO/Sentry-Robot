#include  "drv_can.h"
#include "rc_potocal.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
uint16_t can_cnt_1=0;

float Up_ins_yaw = 0; //上C板yaw值
uint8_t frame_id; //识别码 视觉信息发0 导航信息发1
float nav_vx = 0; //nav前缀的是导航信息
float nav_vy = 0;
float nav_yaw = 0;

void CAN1_Init(void)
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
    can_filter.SlaveStartFilterBank  = 14;          
   
    HAL_CAN_ConfigFilter(&hcan1, &can_filter);        // init can filter
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);		//使能can的FIFO0中断
    HAL_CAN_Start(&hcan1);//启动can1

}

void CAN2_Init( void )
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
    can_filter.SlaveStartFilterBank  = 14;         
   
    HAL_CAN_ConfigFilter(&hcan2, &can_filter);        // init can filter
   	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	  HAL_CAN_Start(&hcan2);//启动can2
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//接受中断回调函数
{
  CAN_RxHeaderTypeDef rx_header;

  if(hcan->Instance == CAN1)
  {
    uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can1 data
		if(rx_header.StdId==0x55)//上C向下C传IMU数据
		{
				if(rx_data[0] == 8)	//校验位
				{
					//Up_ins_yaw = rx_data[1] | (rx_data[2] << 8);			
					memcpy(&Up_ins_yaw,&rx_data[1],4);
					memcpy(&frame_id,&rx_data[5],1);
				}
		}
		
		if(rx_header.StdId==0x56)
		{
				if(rx_data[0] == 9)	//校验位
				{		
					memcpy(&nav_vx,&rx_data[1],4);
				}
		}
		
		if(rx_header.StdId==0x57)
		{
				if(rx_data[0] == 10)	//校验位
				{		
					memcpy(&nav_vy,&rx_data[1],4);
				}
		}
		
		if(rx_header.StdId==0x58)
		{
				if(rx_data[0] == 11)	//校验位
				{		
					memcpy(&nav_yaw,&rx_data[1],4);
				}
		}
  }
	 if(hcan->Instance == CAN2)
  {	//
		uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can2 data
		if ((rx_header.StdId >= 0x201)//201-207
   && (rx_header.StdId <  0x208))                  // 判断标识符，标识符为0x200+ID
  {
    uint8_t index = rx_header.StdId - 0x201;                  // get motor index by can_id
     motor_info_chassis[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
     motor_info_chassis[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
     motor_info_chassis[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
     motor_info_chassis[index].temp           =   rx_data[6];
  }
  }

}


void can_remote(uint8_t sbus_buf[],uint8_t can_send_id)//调用can来发送遥控器数据
{
  CAN_TxHeaderTypeDef tx_header;
    
  tx_header.StdId = can_send_id;//如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf,(uint32_t*)CAN_TX_MAILBOX0);
}

void set_motor_current_can2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x1ff);//如果id_range==0则等于0x200,id_range==1则等于0x1ff（ID号）
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