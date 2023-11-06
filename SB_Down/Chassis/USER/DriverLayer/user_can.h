#include "main.h"

#ifndef __USER_CAN_H__
#define __USER_CNA_H__
void CAN1_Init();
void CAN2_Init();
void can_cmd_send_3508(int motor1,int motor2,int motor3,int motor4);
void can_cmd_send_6020(int motor1,int motor2,int motor3,int motor4);
void can_remote(uint8_t sbus_buf[],uint8_t can_send_id);

typedef struct
{
	double angle;
	int16_t speed;
	uint16_t tor_current;
	uint16_t temperture;
}motor_info;

#endif