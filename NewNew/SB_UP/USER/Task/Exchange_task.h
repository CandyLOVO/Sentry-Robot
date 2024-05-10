#ifndef __EXCHANGE_TASK_H__
#define __EXCHANGE_TASK_H__

#include "main.h"

void Exchange_Task(void * argument);

extern void vision_value(void);
extern void yaw_value(void);

typedef struct
{
  uint8_t header;
	uint8_t L_tracking;
	uint8_t R_tracking;
	uint8_t M_tracking;
	uint8_t L_shoot;
	uint8_t R_shoot;
	float yaw;
	float L_yaw; //自瞄所需yaw到达的角度
	float L_pitch; //自瞄所需pitch到达的角度
	float R_yaw; //自瞄所需yaw到达的角度
	float R_pitch; //自瞄所需pitch到达的角度
//	uint8_t naving; //导航标志位，为1导航
//	float nav_vx; //导航中x的速度
//	float nav_vy; //导航中y的速度
	uint16_t checksum;
}receive_vision; //视觉通信接收结构体

typedef struct
{
	uint8_t header;
	uint8_t color; //1-red 2-blue
	float yaw;
	float L_yaw;
  float L_pitch; //角度制 
	float R_yaw;	
	float R_pitch;
	uint16_t checksum;
	uint8_t ending;
}transmit_vision; //视觉通信发送结构体

#endif
