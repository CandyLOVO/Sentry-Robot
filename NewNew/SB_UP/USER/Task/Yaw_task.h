#ifndef __YAW_TASK_H__
#define __YAW_TASK_H__

#include "main.h"

void Yaw_Task(void const * argument);

static void Yaw_init(void);
float motor_value(int16_t k, int16_t n);
void yaw_control_L(int16_t max_angle, int16_t min_angle);
void yaw_control_R(int16_t max_angle, int16_t min_angle);

#endif
