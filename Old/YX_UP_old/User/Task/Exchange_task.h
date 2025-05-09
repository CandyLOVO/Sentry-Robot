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

//================================================发送位域结构体================================================//
typedef struct
{
	uint8_t detect_color : 1;	//从低位开始
	uint8_t reset_tracker : 1;    // 发0
	uint8_t reserved : 6; 
}		pByte_t;	//位域

//================================================ stm32 -> minipc (发送结构体)================================================//
typedef struct
{
	uint8_t header;
	pByte_t official;
  float pitch_L;               // rad       
  float yaw_L;                 // rad
	float pitch_R;
	float yaw_R;
	uint8_t color;
  uint16_t checksum;     // crc16校验位
	uint8_t process;
	uint16_t self_help;
} 	Vision_t; //视觉通信结构体

//================================================ minipc -> stm32 (接收结构体)================================================//
typedef struct
{
  uint8_t header;
	uint8_t tracking_L; //自瞄标志位，为1瞄准
	float yaw_L;
	float pitch_L;
	uint8_t tracking_R;
	float yaw_R; //自瞄所需yaw到达的角度
	float pitch_R; //自瞄所需pitch到达的角度
	uint8_t naving; //导航标志位，为1导航
	float nav_vx; //导航中x的速度
	float nav_vy; //导航中y的速度
	float distance_L;
	float distance_R;
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

//================================================电机追踪结构体================================================//
//储存的是已经视觉解算成功的视觉数据
typedef struct
{
	float yaw;
	float pitch;
} volatile Chase_t;	//传输给电机的值,这个是需要到达的值

//================================================官方裁判系统的值存储，以及哨兵的一些状态量================================================//
typedef struct
{
	uint8_t foe_flag;		//视觉检测标志位，1为检测成功，0为未检测到装甲板（会有视觉暂留）
	uint8_t foe_count;	//视觉停留计数器
	uint8_t Flag_progress;	//裁判系统比赛进程数据
	uint8_t Flag_judge;	//红蓝方检测
	uint8_t Flag_shoot;	//射击标识位（无暂留）
	uint16_t Cooling_heat;//枪管热量
} Sentry_t;


extern volatile uint8_t rx_len_uart1;  //接收一帧数据的长度
extern volatile uint8_t recv_end_flag_uart1; //一帧数据接收完成标志
extern uint8_t rx_buffer[100];  //接收数据缓存数组

extern volatile Chase_t chase;	//赋予电机追踪的数据结构体
extern remote_flag_t remote;	//键盘按键读取(结构体)
extern Sentry_t Sentry;	//哨兵状态量和裁判系统数据结构体
	
extern void Vision_read(uint8_t rx_buffer[]);



#endif