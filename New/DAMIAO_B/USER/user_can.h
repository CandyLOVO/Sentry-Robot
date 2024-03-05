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
{	uint8_t Fire_Flag ;  //视觉识别标志
	uint16_t speed_limit ; //射速限制
	uint16_t heat_limit ; //热量上限
	uint16_t cooling_rate; //热量每秒冷却值
	uint16_t shooter_heat; //实时枪管热量
	uint8_t shoot_rate; //实时射频
	float shoot_speed; //实时射速 
}Shooter_t;

extern motor_info motor_m3508[6];/*摩擦轮电机 id 1~4*/
extern motor_info motor_m2006[8];/*拨盘电机 id 5~6*/

#endif
