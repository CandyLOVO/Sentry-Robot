#ifndef __LAUNCH_TASK_H__
#define __LAUNCH_TASK_H__

#include "main.h"
typedef struct
{	uint8_t Fire_Flag ;  //�Ӿ�ʶ���־
	uint16_t speed_limit ; //��������
	uint16_t heat_limit ; //��������
	uint16_t cooling_rate; //����ÿ����ȴֵ
	uint16_t shooter_heat; //ʵʱǹ������
	uint8_t shoot_rate; //ʵʱ��Ƶ
	float shoot_speed; //ʵʱ���� 
	float tim_reversal_begin; //��ת����ʱ��
}Shooter_t;


// ����������������������ֵķ�������
void Launch_Task(void * argument);

// �ⲿ�������������ڳ�ʼ��Ħ���ۿ���
extern void friction_init(void);
// �ⲿ������������������Ħ����
extern void mocalun_start(void);
// �ⲿ��������������ֹͣĦ����
extern void mocalun_stop(void);

#endif
