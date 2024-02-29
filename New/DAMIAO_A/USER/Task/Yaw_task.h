#ifndef YAW_TASK_H
#define YAW_TASK_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
#include "struct_typedef.h"
#include "PID.h"
#include "remote_control.h"
#include "handle_value.h"
#include "Motor.h"
#include "Can_user.h"
#include "MF_User.h"
#include "Exchange_task.h"

#define Yaw_sita_weight 0.5f 		//遥控器控制位置环权重
	 
extern float Yaw_middle_c;	//一级云台yaw(只有绝对坐标)
extern float Yaw_left_c;	//现在时刻左脑袋的yaw（相对坐标）
extern float Yaw_right_c;
extern float Yaw_left;
extern float Yaw_right;
	 
//定义函数
void Yaw_task(void const *pvParameters);
static void Yaw_can_send();

#ifdef __cplusplus
}
#endif

#endif