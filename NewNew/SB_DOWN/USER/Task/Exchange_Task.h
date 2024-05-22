#ifndef __EXCHANGE_TASK_H__
#define __EXCHANGE_TASK_H__

#include "main.h"

void Exchange_Task(void const * argument);

extern void RS485_Trans(void);

typedef struct
{
	uint8_t header;
  uint8_t Flag_progress;
  uint8_t color;
  uint16_t projectile_allowance_17mm;
	uint16_t remaining_gold_coin;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_num; 
	uint16_t red_7_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_7_HP;
	uint16_t blue_outpost_HP;  
	uint16_t blue_base_HP;
	uint16_t checksum; 
	float R_yaw;	
	float R_pitch; 
	double yaw12;
	uint8_t ending;
}Tx_naving;

typedef struct
{
	uint8_t header;
  uint8_t naving;
	uint8_t poing;
	float yaw_target;
  float nav_x;
  float nav_y;
  uint32_t sentry_decision;
	uint8_t sentry_decision_buffer[4];
	
	uint8_t R_tracking;
	uint8_t R_shoot;
	float yaw_From_R;
	float R_yaw;
	float R_pitch;
	uint8_t target_shijue; //击打目标，从白头发给黑头 
  uint16_t checksum;
}Rx_naving;

#endif
