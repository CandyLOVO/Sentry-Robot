#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "struct_typedef.h"
#include "chassis_task.h"
#include "motor.h"

void CAN1_Init(void);
void CAN2_Init( void );
void can_remote(uint8_t sbus_buf[],uint8_t can_send_id);

void set_motor_current_can2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);

extern float Up_ins_yaw;	//上C板yaw值，声明给其他文件使用

#endif