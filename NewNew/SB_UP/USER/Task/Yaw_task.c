#include "Yaw_task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "fdcan.h"
#include "main.h"

uint8_t can_send_data[8];
extern motor_info motor[8];

void Yaw_Task(void const * argument)
{
	can_send_data[0] = (1000>>8)&0xff;
	can_send_data[1] = 1000&0xff;

  for(;;)
  {
		osDelay(1);
  }
}

//================================================YAW��PID������Ŀ��IMU��ʼ��================================================//
//static void Yaw_init()
//{
//	pid_init(&motor_pid_can_2[7],6000,2,0,72000,72000); //9025����ٶȻ�
//	pid_init(&motor_pid_sita_can_2[7],3,0,50,72000,72000); //9025����ǶȻ�
//	
////�ϳ���PID Ӳ��һ��
////	pid_init(&motor_pid_can_2[0],200,0.01,0,30000,30000); //��ͷ�ٶȻ�
////	pid_init(&motor_pid_sita_can_2[0],25,0,10,30000,30000); //��ͷ�ǶȻ�
////	
////	pid_init(&motor_pid_can_2[1],200,0.01,0,30000,30000); //��ͷ�ٶȻ�
////	pid_init(&motor_pid_sita_can_2[1],25,0,10,30000,30000); //��ͷ�ǶȻ�
//	
////������PID
//	pid_init(&motor_pid_can_2[0],150,0.01,0,30000,30000); //��ͷ�ٶȻ�
//	pid_init(&motor_pid_sita_can_2[0],10,0,5,30000,30000); //��ͷ�ǶȻ�
//	
//	pid_init(&motor_pid_can_2[1],150,0.01,0,30000,30000); //��ͷ�ٶȻ�
//	pid_init(&motor_pid_sita_can_2[1],10,0,5,30000,30000); //��ͷ�ǶȻ�
//	
//	Encoder_MF_read(motor_info_can_2[7].can_id);//9025��ȡ��ǰ������ֵ
//	Yaw_value = MF_value(Init_encoder_middle , motor_info_can_2[7].rotor_angle , 65535); //��9025����ֵת����-180~0��0~180
//	
//	Yaw_left = -motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle); //��6020����ֵת����-180~0��0~180
//	Yaw_right = -motor_value(Init_encoder_right,motor_info_can_2[1].rotor_angle);
//	target_yaw_middle = Yaw_middle_c;
//	target_yaw_left = Yaw_left;
//	target_yaw_right = Yaw_right;
//}
