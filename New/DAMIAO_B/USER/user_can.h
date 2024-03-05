#include "main.h"

#ifndef __USER_CAN_H__
#define __USER_CAN_H__

void CAN1_Init(void);
void CAN2_Init(void);
void can_send_mocalun(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
void can_send_bopan(int16_t motor1,int16_t motor2);
//void can_remote(uint8_t sbus_buf[],uint8_t can_send_id);

typedef struct
{
	uint16_t angle;
	int16_t speed;
	int16_t tor_current;
	uint16_t temperture;
	int16_t set_v,send_I;
}motor_info;



typedef struct
{	uint8_t Fire_Flag ;  //�Ӿ�ʶ���־
	uint16_t speed_limit ; //��������
	uint16_t heat_limit ; //��������
	uint16_t cooling_rate; //����ÿ����ȴֵ
	uint16_t shooter_heat; //ʵʱǹ������
	uint8_t shoot_rate; //ʵʱ��Ƶ
	float shoot_speed; //ʵʱ���� 
}Shooter_t;

extern motor_info motor_m3508[6];/*Ħ���ֵ�� id 1~4*/
extern motor_info motor_m2006[8];/*���̵�� id 5~6*/

#endif
