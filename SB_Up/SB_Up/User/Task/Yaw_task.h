#ifndef YAW_TASK_H
#define YAW_TASK_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
#include "struct_typedef.h"
#include "INS_task.h"
#include "PID.h"
#include "remote_control.h"
#include "handle_value.h"
#include "Motor.h"
	 
//¶¨Òåº¯Êý
void Yaw_task(void const *pvParameters);

#ifdef __cplusplus
}
#endif

#endif