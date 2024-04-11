#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"

void Chassis_Task(void const * argument);

extern void task_init(void);
extern void Yaw_Diff(void);
#endif
