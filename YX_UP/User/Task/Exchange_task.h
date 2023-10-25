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

void Exchange_task(void const * argument);

typedef struct
{
	uint8_t detect_color : 1;	//�ӵ�λ��ʼ
	uint8_t reset_tracker : 1;    // ��0
	uint8_t reserved : 6; 
}		pByte_t;	//λ��
		
typedef struct
{
	uint8_t header;
	pByte_t official;
	float roll;                // rad ��0
  float pitch;               // rad       
  float yaw;                 // rad
  float aim_x;               // ��0.5
  float aim_y;               // ��0.5
  float aim_z;               // ��0.5
  uint16_t checksum;     // crc16У��λ 	
} 	Vision_t; //�Ӿ�ͨ�Žṹ��

typedef __packed struct
{
		__packed struct
	{
		uint16_t r : 1;//�ӵ�λ��ʼ
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
	
} 	remote_flag_t; //�������ݻ�ȡ

extern uint8_t foe_flag;
extern remote_flag_t remote;
	
extern volatile uint8_t rx_len_uart1;  //����һ֡���ݵĳ���
extern volatile uint8_t recv_end_flag_uart1; //һ֡���ݽ�����ɱ�־
extern int16_t Yaw_minipc;
extern int16_t Pitch_minipc;
extern float Yaw_minipc_fp;
extern float Pitch_minipc_fp;
extern uint8_t rx_buffer[100];
extern int16_t mouse_x;
extern int16_t mouse_y;

void Get_keyboard();

#endif