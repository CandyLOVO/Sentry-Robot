#ifndef FRICTION_TASK_H
#define FRICTION_TASK_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "Exchange_task.h" 
#include "Motor.h"
	 
//===============================================±äÁ¿ÉùÃ÷================================================//
extern uint8_t bopan_reversal_flag;
void Friction_task(void const * argument);
	 
#ifdef __cplusplus
}
#endif

#endif