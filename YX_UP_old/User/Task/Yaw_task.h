#ifndef YAW_TASK_H
#define YAW_TASK_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "Exchange_task.h"
#include "main.h"
#include "pid.h"
#include "struct_typedef.h"
#include "remote_control.h"
#include "INS_task.h"
#include "Motor.h"

//================================================宏定义================================================//
#define Rotate_gain 1.23f		//角速度前馈控制映射权重
#define Chassis_R	30.0f		//车自转半径
#define Chassis_r 7.5f		//麦轮半径
#define valve 20		//遥控器阈值(死区)
#define base 1024		//遥控器的回中值
#define base_max 1684		//遥控器最大值
#define base_min 364		//遥控器最小值
#define angle_valve 1		//角度阈值，在这个范围内就不去抖动了(锁定yaw轴时使用)
#define mouse_x_valve 10.0f		//鼠标控制阈值
#define mouse_x_weight 0.5f		//鼠标映射权重
#define Yaw_sita_weight 0.5f 		//遥控器控制位置环权重
#define Yaw_minipc_weight 1.75f		//视觉跟随映射权重(速度版)
#define Yaw_minipc_sita_weight 0.003f	//视觉跟随映射权重(位置版)

//================================================声明================================================//
extern int16_t Rotate_w;
extern int16_t Rotate_W;
extern fp32 target_yaw;
extern int8_t Update_yaw_flag;
extern float target_yaw;

//定义函数
void Yaw_task(void const *pvParameters);

#ifdef __cplusplus
}
#endif

#endif