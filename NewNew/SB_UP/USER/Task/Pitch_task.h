#ifndef __PITCH_TASK_H__
#define __PITCH_TASK_H__

#include "main.h"

void Pitch_Task(void * argument);

static void Pitch_init(void);
extern void pitch_control_L(int16_t max_angle, int16_t min_angle);
extern void pitch_control_R(int16_t max_angle, int16_t min_angle);
extern void pitch_finding(int16_t max_angle, int16_t min_angle);

#endif
