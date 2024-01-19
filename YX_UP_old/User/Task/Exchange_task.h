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

//================================================����λ��ṹ��================================================//
typedef struct
{
	uint8_t detect_color : 1;	//�ӵ�λ��ʼ
	uint8_t reset_tracker : 1;    // ��0
	uint8_t reserved : 6; 
}		pByte_t;	//λ��

//================================================ stm32 -> minipc (���ͽṹ��)================================================//
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

//================================================ minipc -> stm32 (���սṹ��)================================================//
typedef struct
{
  uint8_t header;
	uint8_t tracking; //�����־λ��Ϊ1��׼
	uint8_t naving; //������־λ��Ϊ1����
	float yaw; //��������yaw����ĽǶ�
	float pitch; //��������pitch����ĽǶ�
	float nav_vx; //������x���ٶ�
	float nav_vy; //������y���ٶ�
	uint16_t checksum;
} vision_receive_t;

//================================================ң���������̽���ṹ��================================================//
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

//================================================���׷�ٽṹ��================================================//
//��������Ѿ��Ӿ�����ɹ����Ӿ�����
typedef struct
{
	float yaw;
	float pitch;
} volatile Chase_t;	//����������ֵ,�������Ҫ�����ֵ

//================================================�ٷ�����ϵͳ��ֵ�洢���Լ��ڱ���һЩ״̬��================================================//
typedef struct
{
	uint8_t foe_flag;		//�Ӿ�����־λ��1Ϊ���ɹ���0Ϊδ��⵽װ�װ壨�����Ӿ�������
	uint8_t foe_count;	//�Ӿ�ͣ��������
	uint8_t Flag_progress;	//����ϵͳ������������
	uint8_t Flag_judge;	//���������
	uint8_t Flag_shoot;	//�����ʶλ����������
} Sentry_t;


extern volatile uint8_t rx_len_uart1;  //����һ֡���ݵĳ���
extern volatile uint8_t recv_end_flag_uart1; //һ֡���ݽ�����ɱ�־

extern Chase_t chase;	//������׷�ٵ����ݽṹ��
extern remote_flag_t remote;	//���̰�����ȡ(�ṹ��)
extern Sentry_t Sentry;	//�ڱ�״̬���Ͳ���ϵͳ���ݽṹ��
	
extern void Vision_read(uint8_t rx_buffer[]);



#endif