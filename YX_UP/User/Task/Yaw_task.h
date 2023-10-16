#ifndef YAW_TASK_H
#define YAW_TASK_H

#include "INS_task.h"
#include "pid.h"
#include "struct_typedef.h"
#include "remote_control.h"

//声明一些其他文件的全局变量
extern int16_t mouse_x;		//鼠标控制量(来自裁判系统)
extern ins_data_t ins_data;		//这是陀螺仪解算出来的数据
extern RC_ctrl_t rc_ctrl;		//遥控器数据

//定义函数
void Yaw_task(void const *pvParameters);
#endif