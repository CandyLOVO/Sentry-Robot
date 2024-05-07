#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"

void Chassis_Task(void const * argument);

extern void task_init(void);
extern void Yaw_Diff(void);
extern void chassis_calculate(int16_t x, int16_t y);
extern void chassis_poing(int16_t x, int16_t y);
extern  void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);
static void Chassis_Power_Limit(double Chassis_pidout_target_limit);
#endif
