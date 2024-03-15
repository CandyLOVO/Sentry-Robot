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

//================================================ stm32 -> minipc (���ͽṹ��)================================================//
typedef struct
{
	uint8_t header;
	uint8_t color; //1-red 2-blue
  float L_pitch; //�Ƕ���     
  float L_yaw;
	float R_pitch;
	float R_yaw;
  uint16_t checksum_L;
	uint16_t checksum_R; 	
	uint8_t ending;
} 	Vision_t; //�Ӿ�ͨ�ŷ��ͽṹ��

//================================================ minipc -> stm32 (���սṹ��)================================================//
typedef struct
{
  uint8_t header;
	uint8_t L_tracking;
	uint8_t R_tracking;
	uint8_t L_shoot;
	uint8_t R_shoot;
	float L_chase_pitch; //��������pitch����ĽǶ�
	float L_chase_yaw; //��������yaw����ĽǶ�
	float R_chase_pitch; //��������pitch����ĽǶ�
	float R_chase_yaw; //��������yaw����ĽǶ�
	float M_chase_yaw;
	uint8_t naving; //������־λ��Ϊ1����
	float nav_vx; //������x���ٶ�
	float nav_vy; //������y���ٶ�
	float L_distance;	//��ͷʶ�𵽵�Ŀ�����(��λ��m)
	float R_distance; //��ͷʶ�𵽵�Ŀ�����
	uint16_t checksum;
} Vision_receive_t;

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

//================================================�ٷ�����ϵͳ��ֵ�洢���Լ��ڱ���һЩ״̬��================================================//
typedef struct
{
	uint8_t Flag_mode;	//Ŀǰ������ģʽ
	uint8_t Remote_mode;	//ң����ģʽ
	uint8_t L_Flag_foe;		//�Ӿ�����־λ��1Ϊ���ɹ���0Ϊδ��⵽װ�װ�
	uint8_t R_Flag_foe;		
	uint8_t L_Flag_yaw_direction;		//Ѳ��ת������0ΪֹͣѲ����1Ϊ��ǰת����2Ϊ���ת��
	uint8_t R_Flag_yaw_direction;		
	uint8_t L_Flag_pitch_direction;		//Ѳ��ת������0ΪֹͣѲ����1Ϊ����ת����2Ϊ����ת��
	uint8_t R_Flag_pitch_direction;	
	uint8_t Flag_progress;	//����ϵͳ������������
	uint8_t Flag_judge;	//��������⣬��0Ϊ����ϵͳ���ˣ���1Ϊ�ҷ��Ǻ�ɫ������2Ϊ�ҷ�����ɫ��
	uint8_t Flag_armour;	//�ܻ����װ�װ��ţ�δ������ʱΪ0
	uint16_t Time_remain;		//����ʣ��ʱ��
	
	uint16_t Myself_remain_HP;	//��������ʣ��Ѫ��
	uint16_t Myself_17mm_cooling_heat_id1;		//ʵʱǹ��1����
	uint16_t Myself_17mm_cooling_heat_id2;		//ʵʱǹ��2����
	
} Sentry_t;


extern volatile uint8_t rx_len_uart4;  //����һ֡���ݵĳ���
extern volatile uint8_t recv_end_flag_uart4; //һ֡���ݽ�����ɱ�־
extern uint8_t rx_buffer[100];  //�������ݻ�������
extern remote_flag_t remote;	//���̰�����ȡ(�ṹ��)
extern Sentry_t Sentry;	//�ڱ�״̬���Ͳ���ϵͳ���ݽṹ��
extern Vision_t vision;
extern Vision_receive_t vision_receive;

void Vision_read(uint8_t rx_buffer[]);



#endif