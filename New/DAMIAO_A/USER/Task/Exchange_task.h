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
#include "Can_user.h"
#include "SolveTrajectory.h"
#include "Motor.h"
#include "Yaw_task.h"
#include "Pitch_task.h"
#include "CRC.h"

#define BUFFER_SIZE 100

void Exchange_task(void const * argument);

//================================================ stm32 -> minipc (发送结构体)================================================//
typedef struct
{
	uint8_t header;
	uint8_t color; //1-red 2-blue
  float L_pitch; //角度制     
  float L_yaw;
	float R_pitch;
	float R_yaw;
  uint16_t checksum_L;
	uint16_t checksum_R; 	
	uint8_t ending;
} 	Vision_t; //视觉通信发送结构体

//================================================ minipc -> stm32 (接收结构体)================================================//
typedef struct
{
  uint8_t header;
	uint8_t L_tracking;
	uint8_t R_tracking;
	uint8_t L_shoot;
	uint8_t R_shoot;
	float L_chase_pitch; //自瞄所需pitch到达的角度
	float L_chase_yaw; //自瞄所需yaw到达的角度
	float R_chase_pitch; //自瞄所需pitch到达的角度
	float R_chase_yaw; //自瞄所需yaw到达的角度
	float M_chase_yaw;
	uint8_t naving; //导航标志位，为1导航
	float nav_vx; //导航中x的速度
	float nav_vy; //导航中y的速度
	float L_distance;	//左头识别到的目标距离(单位：m)
	float R_distance; //右头识别到的目标距离
	uint16_t checksum;
} Vision_receive_t;

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
	uint8_t Flag_mode;	//目前的自瞄模式
	uint8_t Remote_mode;	//遥控器模式
	uint8_t L_Flag_foe;		//视觉检测标志位，1为检测成功，0为未检测到装甲板
	uint8_t R_Flag_foe;		
	uint8_t L_Flag_yaw_direction;		//巡航转动方向，0为停止巡航，1为向前转动，2为向后转动
	uint8_t R_Flag_yaw_direction;		
	uint8_t L_Flag_pitch_direction;		//巡航转动方向，0为停止巡航，1为向上转动，2为向下转动
	uint8_t R_Flag_pitch_direction;	
	uint8_t Flag_progress;	//裁判系统比赛进程数据
	uint8_t Flag_judge;	//红蓝方检测，置0为裁判系统寄了，置1为我方是红色方，置2为我方是蓝色方
	uint8_t Flag_armour;	//受击打的装甲板编号，未被击打时为0
	uint16_t Time_remain;		//比赛剩余时间
	
	uint16_t Myself_remain_HP;	//本机器人剩余血量
	uint16_t Myself_17mm_cooling_heat_id1;		//实时枪管1热量
	uint16_t Myself_17mm_cooling_heat_id2;		//实时枪管2热量
	
} Sentry_t;


extern volatile uint8_t rx_len_uart4;  //接收一帧数据的长度
extern volatile uint8_t recv_end_flag_uart4; //一帧数据接收完成标志
extern uint8_t rx_buffer[100];  //接收数据缓存数组
extern remote_flag_t remote;	//键盘按键读取(结构体)
extern Sentry_t Sentry;	//哨兵状态量和裁判系统数据结构体
extern Vision_t vision;
extern Vision_receive_t vision_receive;

void Vision_read(uint8_t rx_buffer[]);



#endif