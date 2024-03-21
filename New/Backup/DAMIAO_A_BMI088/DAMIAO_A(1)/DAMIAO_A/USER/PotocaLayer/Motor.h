#ifndef MOTOR_H
#define MOTOR_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "main.h"
//================================================����ṹ��================================================//
typedef struct
{
    uint16_t can_id;		//ID��
    int32_t  set_voltage;		//������Ϣ
    uint16_t rotor_angle;		//���ڵĽǶ�
    int16_t  rotor_speed;		//���ڵ�ת��
    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�
	 uint16_t last_angle;
}moto_info_t;

//================================================����ṹ������================================================//
extern float target_speed[8];	//����׷���ٶ�
extern float target_speed_can_2[8];
extern moto_info_t motor_info[8];		//��������7���ֽ�
extern moto_info_t motor_info_can_2[8];

	 
#ifdef __cplusplus
}
#endif

#endif