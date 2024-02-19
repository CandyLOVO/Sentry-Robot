#ifndef EXCHANGE_TASK_H
#define EXCHANGE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "INS_task.h"
#include "Can_user.h"
#include "SolveTrajectory.h"
#include "Motor.h"
#include "Yaw_task.h"
#include "Pitch_task.h"

#define BUFFER_SIZE 100

void Exchange_task(void const * argument);

//================================================ stm32 -> minipc (发送结构体)================================================//
typedef struct
{
	uint8_t header;
  float L_pitch;	//角度制     
  float L_yaw;
	float R_pitch;
	float R_yaw;
  uint16_t checksum;     // crc16校验位 	
} 	Vision_t; //视觉通信发送结构体

//================================================ minipc -> stm32 (接收结构体)================================================//
typedef struct
{
  uint8_t header;
	float L_chase_pitch; //自瞄所需pitch到达的角度
	float L_chase_yaw; //自瞄所需yaw到达的角度
	float R_chase_pitch; //自瞄所需pitch到达的角度
	float R_chase_yaw; //自瞄所需yaw到达的角度
	uint8_t naving; //导航标志位，为1导航
	float nav_vx; //导航中x的速度
	float nav_vy; //导航中y的速度
	uint16_t checksum;
} vision_receive_t;

//================================================遥控器及键盘解算结构体================================================//
typedef __packed struct
{
		__packed struct
	{
		uint16_t r : 1;//从低位开始
		uint16_t f : 1;
		uint16_t g : 1;
		uint16_t z : 1;
		uint16_t x : 1;
		uint16_t c : 1;
		uint16_t v : 1;
		uint16_t b : 1;
		uint16_t w : 1;
		uint16_t s : 1;
		uint16_t a : 1;
		uint16_t d : 1;
		uint16_t q : 1;
		uint16_t e : 1;
		uint16_t shift : 1;
		uint16_t ctrl : 1;
	} key;
	
		__packed struct
	{
		uint16_t press_left;
		uint16_t press_right;
		uint16_t x;
		uint16_t y;
	}	mouse;
	
} 	remote_flag_t; //键盘数据获取

//================================================官方裁判系统的值存储，以及哨兵的一些状态量================================================//
typedef struct
{
	uint8_t Flag_mode;	//目前的模式
	uint8_t L_Flag_foe;		//视觉检测标志位，1为检测成功，0为未检测到装甲板
	uint8_t R_Flag_foe;		
	uint8_t Flag_progress;	//裁判系统比赛进程数据
	uint8_t Flag_judge;	//红蓝方检测，置0为裁判系统寄了，置1为我方是红色方，置2为我方是蓝色方
} Sentry_t;


extern volatile uint8_t rx_len_uart1;  //接收一帧数据的长度
extern volatile uint8_t recv_end_flag_uart1; //一帧数据接收完成标志
extern uint8_t rx_buffer[100];  //接收数据缓存数组
extern remote_flag_t remote;	//键盘按键读取(结构体)
extern Sentry_t Sentry;	//哨兵状态量和裁判系统数据结构体
	
void Vision_read(uint8_t rx_buffer[]);



#endif