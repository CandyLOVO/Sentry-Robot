#include "can_user.h"
#include "fdcan.h"
#include "string.h"
#include "tim.h"
#include "Launch_task.h"
#include "Exchange_task.h"

motor_info motor[8];
motor_info motor_friction[8];
RC_ctrl_t rc_ctrl;
uint8_t flag_heart;
uint8_t target_shijue;

extern Shooter_t Shooter_L;
extern Shooter_t Shooter_R;
extern uint16_t time_delay;
extern uint8_t flag_suo;
extern receive_vision Rx_vision;
extern transmit_vision Tx_vision;

FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

FDCAN_RxHeaderTypeDef RxHeader3;
uint8_t g_Can3RxData[64];

void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */	
	sFilterConfig.IdType = FDCAN_STANDARD_ID;//标准ID，扩展ID不接收
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; // 
  sFilterConfig.FilterID2 = 0x00000000; // 
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
/* 全局过滤设置 */
/* 接收到消息ID与标准ID过滤不匹配，不接受 */
/* 接收到消息ID与扩展ID过滤不匹配，不接受 */
/* 过滤标准ID远程帧 */ 
/* 过滤扩展ID远程帧 */ 
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

	/* 开启RX FIFO0的新数据中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCAN3_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;//标准ID，扩展ID不接收
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; // 
  sFilterConfig.FilterID2 = 0x00000000; // 
  if(HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;                 // CAN ID
  TxHeader.IdType =  FDCAN_STANDARD_ID ;        
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
  if(len<=8)	
	{
	  TxHeader.DataLength = len<<16;     // 发送长度：8byte
	}
	else  if(len==12)	
	{
	   TxHeader.DataLength =FDCAN_DLC_BYTES_12;
	}
	else  if(len==16)	
	{
	  TxHeader.DataLength =FDCAN_DLC_BYTES_16;
	
	}
  else  if(len==20)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_20;
	}		
	else  if(len==24)	
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_24;	
	}else  if(len==48)
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_48;
	}else  if(len==64)
   {
		 TxHeader.DataLength =FDCAN_DLC_BYTES_64;
	 }
											
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//比特率切换关闭，
  TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            // CAN2.0
  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
  TxHeader.MessageMarker = 0;//消息标记

   // 发送CAN指令
  if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
        // 发送失败处理
//       Error_Handler();
		MX_TIM3_Init();
		FDCAN3_Config();
  }
	 return 0;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN1)
    {
      /* Retrieve Rx messages from RX FIFO0 */
			memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//接收前先清空数组	
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
			if((RxHeader1.Identifier >= 0x205) && (RxHeader1.Identifier <= 0x208))
			{
				uint8_t index = RxHeader1.Identifier - 0x205;
				motor[index].angle = ((g_Can1RxData[0] << 8) | g_Can1RxData[1]);
				motor[index].speed = ((g_Can1RxData[2] << 8) | g_Can1RxData[3]);
				motor[index].tor_current = ((g_Can1RxData[4] << 8) | g_Can1RxData[5]);
				motor[index].temperture = g_Can1RxData[6];
			}
	  }
		
		if(hfdcan->Instance == FDCAN3)
    {
      /* Retrieve Rx messages from RX FIFO0 */
			memset(g_Can3RxData, 0, sizeof(g_Can3RxData));	//接收前先清空数组	
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader3, g_Can3RxData);
			if(RxHeader3.Identifier == 0x30) //rc_ctrl.rc.ch[0]~[3]
			{
				rc_ctrl.rc.ch[0] = (g_Can3RxData[0]<<8) | g_Can3RxData[1];
				rc_ctrl.rc.ch[1] = (g_Can3RxData[2]<<8) | g_Can3RxData[3];
				rc_ctrl.rc.ch[2] = (g_Can3RxData[4]<<8) | g_Can3RxData[5];
				rc_ctrl.rc.ch[3] = (g_Can3RxData[6]<<8) | g_Can3RxData[7];
			}
			
			if(RxHeader3.Identifier == 0x31) //rc_ctrl.rc[4]&rc_ctrl.rc.s&rc_ctrl.key
			{
				rc_ctrl.rc.ch[4] = (g_Can3RxData[0]<<8) | g_Can3RxData[1];
				rc_ctrl.rc.s[0] = g_Can3RxData[2];
				rc_ctrl.rc.s[1] = g_Can3RxData[3];
				rc_ctrl.key.v = (g_Can3RxData[4]<<8) | g_Can3RxData[5];
			}
		
			if(RxHeader3.Identifier == 0x32) //rc_ctrl.mouse
			{
				rc_ctrl.mouse.x = (g_Can3RxData[0]<<8) | g_Can3RxData[1];
				rc_ctrl.mouse.y = (g_Can3RxData[2]<<8) | g_Can3RxData[3];
				rc_ctrl.mouse.z = (g_Can3RxData[4]<<8) | g_Can3RxData[5];
				rc_ctrl.mouse.press_l = g_Can3RxData[6];
				rc_ctrl.mouse.press_r = g_Can3RxData[7];
			}
			
			if(RxHeader3.Identifier == 0x36)
			{
				//先收高八位，再收低八位
				Shooter_L.shooter_heat = ((g_Can3RxData[0] << 8) | g_Can3RxData[1]); //枪管1实时热量
				Shooter_R.shooter_heat = ((g_Can3RxData[2] << 8) | g_Can3RxData[3]); //枪管2实时热量
				flag_heart = g_Can3RxData[4]; //受击打装甲板ID
				memcpy(&time_delay, &g_Can3RxData[5], 2);
				flag_suo = g_Can3RxData[7];
			}
			
			if(RxHeader3.Identifier == 0x37)
			{ 
        if(g_Can3RxData[0] == 0x01)
        {	
          Shooter_L.shoot_rate = g_Can3RxData[1];
          memcpy(&Shooter_L.shoot_speed, &g_Can3RxData[2], 4);
        }

        if(g_Can3RxData[0] == 0x02)
        {
          Shooter_R.shoot_rate = g_Can3RxData[1];
          memcpy(&Shooter_R.shoot_speed, &g_Can3RxData[2], 4);
        }
				
				memcpy(&Tx_vision.shoot_speed, &g_Can3RxData[2], 4);
			}
			
			if(RxHeader3.Identifier == 0x39)
			{
				Rx_vision.R_tracking = g_Can3RxData[0];
				Rx_vision.R_shoot = g_Can3RxData[1];
				target_shijue = g_Can3RxData[2];
			}
			
			if(RxHeader3.Identifier == 0x40)
			{
				memcpy(&Rx_vision.R_yaw, &g_Can3RxData[0], 4);
				memcpy(&Rx_vision.R_pitch, &g_Can3RxData[4], 4);
			}
	  }
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN2)
    {
      /* Retrieve Rx messages from RX FIFO1 */
			memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
			
			if((RxHeader2.Identifier >= 0x201) && (RxHeader2.Identifier <= 0x208))
			{
				uint8_t index = RxHeader2.Identifier - 0x201;
				motor_friction[index].angle = ((g_Can2RxData[0] << 8) | g_Can2RxData[1]);
				motor_friction[index].speed = ((g_Can2RxData[2] << 8) | g_Can2RxData[3]);
				motor_friction[index].tor_current = ((g_Can2RxData[4] << 8) | g_Can2RxData[5]);
				motor_friction[index].temperture = g_Can2RxData[6];
			}
    }
  }
}
