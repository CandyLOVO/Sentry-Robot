#ifndef __YAW_TASK_H__
#define __YAW_TASK_H__

#include "main.h"

void Yaw_task(void const * argument);

extern void yaw_init(void);
extern void yaw_control(void);
extern void yaw_finding(void);
extern void yaw_suoing(void);
extern float motor_value(int32_t k, int32_t n, int32_t max);
extern void yaw_forward(void);
#endif
