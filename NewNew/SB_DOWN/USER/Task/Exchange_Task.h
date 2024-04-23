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
	uint8_t ending;
}Tx_naving;

typedef struct
{
	uint8_t header;
  uint8_t naving;
  float nav_x;
  float nav_y;
  uint32_t sentry_decision;
  uint16_t checksum;
}Rx_naving;

#endif
