#include "Can_user.h"
#include "remote_control.h"
#include "Yaw_task.h"
#include "Motor.h"
#include "Exchange_task.h"
//	Can ��һЩ�û�׫д�Ľ��պ���

int16_t Down_pitch;	//����pitch����

//================================================can1������================================================//
void can_1_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // ��ʶ������λģʽ
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//������λ��Ϊ����32λ
  can_filter.FilterIdHigh = 0;//��ʶ���Ĵ��� 
  can_filter.FilterIdLow  = 0;//��ʶ���Ĵ��� 
  can_filter.FilterMaskIdHigh = 0;//���μĴ���
  can_filter.FilterMaskIdLow  = 0;       //���μĴ���   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0����������FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
	HAL_CAN_Start(&hcan1);//����can����װ��can_user_init()����
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);		//ʹ��can��FIFO0�жϣ�Ҳ��װ��can_user_init()����
}

//================================================can2������================================================//
void can_2_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 14;                       // filter 14
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // ��ʶ������λģʽ
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//������λ��Ϊ����32λ
  can_filter.FilterIdHigh = 0;//��ʶ���Ĵ��� 
  can_filter.FilterIdLow  = 0;//��ʶ���Ĵ��� 
  can_filter.FilterMaskIdHigh = 0;//���μĴ���
  can_filter.FilterMaskIdLow  = 0;       //���μĴ���   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0����������FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
	HAL_CAN_Start(&hcan2);//����can����װ��can_user_init()����
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);		//ʹ��can��FIFO0�жϣ�Ҳ��װ��can_user_init()����
}

//================================================can�ص�����(�ж�)================================================//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//�����жϻص�����
{
  CAN_RxHeaderTypeDef rx_header;
	uint8_t             rx_data[8];
//================================================can1����================================================//
  if(hcan->Instance == CAN1)
  {
//================================================ң��������================================================//
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
		
//================================================��Ϣ����================================================//		
		if(rx_header.StdId == 0x54)
		{
			Sentry.Flag_progress = rx_data[0];  //��������
			Sentry.Flag_judge = rx_data[1];  //�ж��ҷ��Ǻ�ɫ��������ɫ��
			Sentry.Flag_armour = rx_data[2];  //�ܻ���װ�װ���
			Sentry.Time_remain = (rx_data[4]<<8) | rx_data[3];  //����ʣ��ʱ��
			//memcpy(&Sentry.Time_remain,&rx_data[3],2);//������ȼ�		
			Sentry.Myself_remain_HP = (rx_data[6]<<8) | rx_data[5];	//��������ʣ��Ѫ��			
		}
		
//		if(rx_header.StdId == 0x55)
//		{
//			Sentry.Myself_remain_HP = (rx_data[1]<<8) | rx_data[0];	//��������ʣ��Ѫ��
//			Sentry.Myself_17mm_cooling_heat_id1 = (rx_data[3]<<8) | rx_data[2];	//ʵʱǹ��1����
//			Sentry.Myself_17mm_cooling_heat_id2 = (rx_data[5]<<8) | rx_data[4];	//ʵʱǹ��2����	
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
	
	
//================================================can2����================================================//
	 if(hcan->Instance == CAN2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
//================================================������ݽ���================================================//		
//����3508��2006������
	if ((rx_header.StdId >= 0x205) && (rx_header.StdId < 0x20B))// �жϱ�ʶ������ʶ��Ϊ0x200+ID
  {
		uint8_t index = rx_header.StdId - 0x205;   
    motor_info_can_2[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info_can_2[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info_can_2[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info_can_2[index].temp           =   rx_data[6];
  }
	
//================================================MF9025��ע��һ�µ�λ��================================================//
int16_t iq;
int16_t speed;
int16_t encoder;	
	if(rx_header.StdId == 0x141)//MF9025���IDΪ1
	{
		if(rx_data[0] == 0xA2)	//����ģʽ�µķ���ֵ
		{
			motor_info_can_2[7].torque_current = (rx_data[2] | (rx_data[3] << 8));//�ȷ��ĵ�λ�ֽ�
			motor_info_can_2[7].rotor_speed = (rx_data[4] | (rx_data[5] << 8));
			motor_info_can_2[7].rotor_angle = (rx_data[6] | (rx_data[7] << 8));
		}
		else if(rx_data[0] == 0x90)	//��ȡ�����������ڳ�ʼ����
		{
			motor_info_can_2[7].rotor_angle = (rx_data[2] | (rx_data[3] <<8));//�ȷ��ĵ�λ�ֽ�
		}
	}
  }
}

//================================================ң�������ݰ�䷢�ͺ���================================================//
void can_remote(uint8_t sbus_buf[],uint32_t id)
{
  CAN_TxHeaderTypeDef tx_header;  
  tx_header.StdId = id;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf,(uint32_t*)CAN_TX_MAILBOX0);
}


//================================================can1��׼���ͺ���================================================//
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x1ff);//���id_range==0�����0x1ff,id_range==1�����0x2ff��ID�ţ�
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (v1>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

//================================================can2��׼���ͺ���================================================//
void set_motor_voltage_can_2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x1ff);//���id_range==0�����0x1ff,id_range==1�����0x2ff��ID�ţ�
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (v1>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
