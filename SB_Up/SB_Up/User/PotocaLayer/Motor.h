#ifndef MOTOR_H
#define MOTOR_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "main.h"
//================================================电机结构体================================================//
typedef struct
{
    uint16_t can_id;		//ID号
    int16_t  set_voltage;		//发送信息
    uint16_t rotor_angle;		//现在的角度
    int16_t  rotor_speed;		//现在的转速
    int16_t  torque_current;		//实际转矩电流
    uint8_t  temp;		//电机温度
}moto_info_t;

//================================================电机结构体声明================================================//
extern float target_speed[7];	//定义追踪速度
extern float target_speed_can_2[7];
extern moto_info_t motor_info[7];		//赋予最大的7个字节
extern moto_info_t motor_info_can_2[7];

	 
#ifdef __cplusplus
}
#endif

#endif