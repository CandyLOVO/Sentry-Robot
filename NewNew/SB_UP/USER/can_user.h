#ifndef __CAN_USER_H__
#define __CAN_USER_H__

#include "main.h"

extern void FDCAN1_Config(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

typedef struct //CAN
{
	uint16_t angle;
	int16_t speed;
	uint16_t tor_current;
	uint16_t temperture;
}motor_info;

#endif
