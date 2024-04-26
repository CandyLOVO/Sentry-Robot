#ifndef __CAN_USER_H__
#define __CAN_USER_H__

#include "main.h"

extern void CAN1_Init(void);
extern void CAN2_Init(void);
extern void can_cmd_send_3508(int motor1,int motor2,int motor3,int motor4);
extern void can_cmd_send_6020(int motor1,int motor2,int motor3,int motor4);
extern void can_remote(uint8_t sbus_buf[],uint8_t can_send_id);
extern void can_cmd_send_5010(uint8_t sbus_buf[]);

typedef struct //CAN
{
	uint16_t angle;
	int16_t speed;
	int16_t tor_current;
	uint8_t temperture;
}motor_info;

typedef struct
{
	int8_t temperture;
	int16_t tor_current;
	int16_t speed;
	uint16_t angle;
}motor_5010_info;

#endif
