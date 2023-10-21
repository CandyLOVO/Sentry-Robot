#ifndef CAN_USER_H
#define CAN_USER_H

#include "main.h"

extern int16_t Down_pitch;	//µ×ÅÌpitchÊý¾Ý
void can_1_user_init(CAN_HandleTypeDef* hcan );
void can_2_user_init(CAN_HandleTypeDef* hcan );
void can_remote(uint8_t sbus_buf[],uint32_t id);
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_voltage_can_2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);

#endif