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
{int16_t Cooling_heat_L,Cooling_heat_R,remainHP;
	int Fire_flag_L,Fire_flag_R,Flag_mode;
}ROBOT;

extern motor_info motor_m3508[6];/*摩擦轮电机 id 1~4*/
extern motor_info motor_m2006[8];/*拨盘电机 id 5~6*/

#endif
