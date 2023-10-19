#include "freertos.h"
#include "can.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "pid.h"
float set = 60; //给定速度
uint16_t output[4];

void Chassis(void const * argument)
{
	pidTypeDef PID_angle[4];
	pidTypeDef PID_speed[4];
	float PID_s[3] = {30,1,0};
//	float set_angle = 1365; //60degrees
//	float Max_out_a = 2000;
//	float Max_iout_a = 2000;
	float Max_out_s = 30000; //电压控制转速，电流控制扭矩
	float Max_iout_s = 30000;
//	float angle[4];
//	float speed[4];
//	float output[4];
	motor_info motor[4];
	for(int i=0;i<4;i++){
		pid_init(&PID_speed[i],PID_s[0],PID_s[1],PID_s[2]);
	}
	
  for(;;)
  {
		for(int i=0;i<4;i++){
//			pid_init(&PID_angle[i],Kp,Ki,Kd);
//			speed[i] = pid_cal(&PID_angle[i],motor[i].angle,set_angle,Max_out_a,Max_iout_a);
//			pid_init(&PID_speed[i],Kp,Ki,Kd);
//			output[i] = pid_cal(&PID_speed[i],motor[i].speed,speed[i],Max_out_s,Max_iout_s);
			output[i] = pid_cal_s(&PID_speed[i],motor[i].speed,set,Max_out_s,Max_iout_s);
		}
		can_cmd_send(output[0],output[1],output[2],output[3]);
		HAL_Delay(10);
    osDelay(1);
  }
}
