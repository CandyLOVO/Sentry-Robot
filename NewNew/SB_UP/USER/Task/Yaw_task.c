#include "Yaw_task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "fdcan.h"
#include "main.h"

uint8_t can_send_data[8];
extern motor_info motor[8];

void Yaw_Task(void const * argument)
{
//	can_send_data[0] = (1000>>8)&0xff;
//	can_send_data[1] = 1000&0xff;
	can_send_data[0] = 0x01;
	can_send_data[1] = 0x02;
	can_send_data[2] = 0x03;
	can_send_data[3] = 0x04;
	can_send_data[4] = 0x05;
	can_send_data[5] = 0x06;

  for(;;)
  {
//		canx_send_data(&hfdcan3, 0x31, can_send_data, 6);
		osDelay(1);
  }
}

//================================================YAW轴PID参数和目标IMU初始化================================================//
//static void Yaw_init()
//{
//	pid_init(&motor_pid_can_2[7],6000,2,0,72000,72000); //9025电机速度环
//	pid_init(&motor_pid_sita_can_2[7],3,0,50,72000,72000); //9025电机角度环
//	
////上场用PID 硬的一批
////	pid_init(&motor_pid_can_2[0],200,0.01,0,30000,30000); //左头速度环
////	pid_init(&motor_pid_sita_can_2[0],25,0,10,30000,30000); //左头角度环
////	
////	pid_init(&motor_pid_can_2[1],200,0.01,0,30000,30000); //右头速度环
////	pid_init(&motor_pid_sita_can_2[1],25,0,10,30000,30000); //右头角度环
//	
////调试用PID
//	pid_init(&motor_pid_can_2[0],150,0.01,0,30000,30000); //左头速度环
//	pid_init(&motor_pid_sita_can_2[0],10,0,5,30000,30000); //左头角度环
//	
//	pid_init(&motor_pid_can_2[1],150,0.01,0,30000,30000); //右头速度环
//	pid_init(&motor_pid_sita_can_2[1],10,0,5,30000,30000); //右头角度环
//	
//	Encoder_MF_read(motor_info_can_2[7].can_id);//9025读取当前编码器值
//	Yaw_value = MF_value(Init_encoder_middle , motor_info_can_2[7].rotor_angle , 65535); //将9025编码值转换到-180~0、0~180
//	
//	Yaw_left = -motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle); //将6020编码值转换到-180~0、0~180
//	Yaw_right = -motor_value(Init_encoder_right,motor_info_can_2[1].rotor_angle);
//	target_yaw_middle = Yaw_middle_c;
//	target_yaw_left = Yaw_left;
//	target_yaw_right = Yaw_right;
//}
