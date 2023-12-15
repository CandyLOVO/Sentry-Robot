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
#include "Can_user.h"
#include "MF_User.h"

#define Yaw_sita_weight 0.5f 		//遥控器控制位置环权重
	 
//定义函数
void Yaw_task(void const *pvParameters);
static void Yaw_can_send();

#ifdef __cplusplus
}
#endif

#endif