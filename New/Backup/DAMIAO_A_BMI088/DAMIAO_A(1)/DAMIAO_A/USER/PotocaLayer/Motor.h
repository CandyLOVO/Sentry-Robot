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
    int32_t  set_voltage;		//发送信息
    uint16_t rotor_angle;		//现在的角度
    int16_t  rotor_speed;		//现在的转速
    int16_t  torque_current;		//实际转矩电流
    uint8_t  temp;		//电机温度
	 uint16_t last_angle;
}moto_info_t;

//================================================电机结构体声明================================================//
extern float target_speed[8];	//定义追踪速度
extern float target_speed_can_2[8];
extern moto_info_t motor_info[8];		//赋予最大的7个字节
extern moto_info_t motor_info_can_2[8];

	 
#ifdef __cplusplus
}
#endif

#endif