#ifndef __YAW_TASK_H__
#define __YAW_TASK_H__

#include "main.h"

void Yaw_Task(void * argument);

static void Yaw_init(void);
extern float motor_value(int16_t k, int16_t n);
extern void yaw_control_L(float max_angle, float min_angle);
extern void yaw_control_R(float max_angle, float min_angle);
extern void yaw_finding_L(float max_angle, float min_angle);
extern void yaw_finding_R(float max_angle, float min_angle);

#endif
