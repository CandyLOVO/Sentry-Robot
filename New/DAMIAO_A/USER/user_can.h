#include "main.h"

#ifndef __USER_CAN_H__
#define __USER_CAN_H__

void CAN1_Init(void);
void CAN2_Init(void);
void can_send_6020(int motor1,int motor2,int motor3,int motor4);
void can_send_9025(int32_t speedControl);
void can_remote(uint8_t sbus_buf[],uint8_t can_send_id);

typedef struct
{
	uint16_t angle;
	int16_t speed;
	uint16_t tor_current;
	uint16_t temperture;
}motor_info;

#endif
