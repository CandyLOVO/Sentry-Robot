#include "MF_User.h"

//================================================겿�MF9025���������================================================//
//�����õ�can_2

//ʾ�����պ�����ע�ⵥλ��
//HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//	if(rx_header.StdId == 0x141)//MF9025���IDΪ1
//	{
//		if(rx_data[0] == 0xA1)	//����ģʽ�µķ���ֵ
//		{
//			motor_info[0].torque_current = (rx_data[2] | (rx_data[3] << 8));//�ȷ��ĵ�λ�ֽ�
//			motor_info[0].rotor_speed = (rx_data[4] | (rx_data[5] << 8));
//			motor_info[0].rotor_angle = (rx_data[6] | (rx_data[7] << 8));
//		}
//	}

//================================================�������================================================//
//para:ID��
void Start_MF_send(int16_t ID)
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x140+ID;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = 0x88;	//�ȷ��߰�λ		
  tx_data[1] = 0x00;
  tx_data[2] = 0x00;	
  tx_data[3] = 0x00;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);
}

//================================================�ջ�ת�ؿ���================================================//
/*��ֵ��Χ-2048~ 2048����Ӧ MF ���ʵ��ת�ص�����Χ-16.5A~16.5A����Ӧ MG ���ʵ��ת�ص�����Χ-33A~33A��
ĸ�ߵ����͵����ʵ��Ť����ͬ������졣*/

//para:ID��,����ֵ�ĵ�ַ
void Current_Control_MF_send(int16_t ID,int16_t iqControl)
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x140+ID;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = 0xA1;	//�ȷ��߰�λ		
  tx_data[1] = 0x00;
  tx_data[2] = 0x00;	
  tx_data[3] = 0x00;
  tx_data[4] = *(uint8_t *)(&iqControl);//iqControl&0xff
  tx_data[5] = *((uint8_t *)(&iqControl)+1);//(iqControl>>8)&0xff
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);
}

//================================================MF����ջ�ת�ؿ��Ƶ�������================================================//
//���룺����ֵ
//��������ƺ�ĵ���ֵ
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

//��MF9025���(ͨ�ð�)�ĽǶ��Գ�ʼ�Ƕ�Ϊ0��ӳ�䵽0~+-180�� (k:�趨�ĳ�ʼ�Ƕȡ�0�� �� n:��Ҫӳ��ĽǶ�)
//��3�������Ǳ��������ֵ
float MF_value(int32_t k, int32_t n, int32_t max)
{
	int32_t middle = (max+1)/2;
	float output;
	if(k>=0 && k<middle){
		if(n>=0 && n<(k+middle)){
			n = k - n;
			output = (float)n * 360.f / (float)max;
			return output;
		}
		else if(n>=(k+middle) && n<max){
			n = max - n + k;
			output = (float)n * 360.f / (float)max;
			return output;
		}
	}
	
	else if(k>=middle && k<max){
		if(n>=0 && n<(k-middle)){
			n = -max + k - n;
			output = (float)n * 360.f / (float)max;
			return output;
		}
		else if(n>=(k-middle) && n<max){
			n = k - n;
			output = (float)n * 360.f / (float)max;
			return output;
		}
	}
}

//================================================ֻ��ȡ������(���ڳ�ʼ��)================================================//
 void Encoder_MF_read(int16_t ID)
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x140+ID;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = 0x90;	//�ȷ��߰�λ		
  tx_data[1] = 0x00;
  tx_data[2] = 0x00;	
  tx_data[3] = 0x00;
  tx_data[4] = 0x00;//iqControl&0xff
  tx_data[5] = 0x00;//(iqControl>>8)&0xff
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
	
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);
}
