#ifndef __LAUNCH_TASK_H__
#define __LAUNCH_TASK_H__

#include "main.h"
typedef struct
{	uint8_t Fire_Flag ;  //视觉识别标志
	uint16_t speed_limit ; //射速限制
	uint16_t heat_limit ; //热量上限
	uint16_t cooling_rate; //热量每秒冷却值
	uint16_t shooter_heat; //实时枪管热量
	uint8_t shoot_rate; //实时射频
	float shoot_speed; //实时射速 
	float tim_reversal_begin; //反转启动时间
}Shooter_t;


// 启动任务函数，负责管理射手的发射任务
void Launch_Task(void * argument);

// 外部函数声明，用于初始化摩擦论控制
extern void friction_init(void);
// 外部函数声明，用于启动摩擦轮
extern void mocalun_start(void);
// 外部函数声明，用于停止摩擦轮
extern void mocalun_stop(void);

#endif
