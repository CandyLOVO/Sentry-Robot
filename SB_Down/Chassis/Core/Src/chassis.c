#include "freertos.h"
#include "can.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "pid.h"

int16_t set_angle = 1365; //60度


void Chassis(void const * argument)
{
	pidTypeDef PID_angle[4];
	pidTypeDef PID_speed[4];
	float PID_s[3] = {1,0.01,0};
	float PID_a[3] = {30,0,0.01};
//	int16_t set_angle = 1365; //60度
	int16_t Max_out_a = 8191;
	int16_t Max_iout_a = 8191;
	
	int16_t Max_out_s = 30000; //电压控制转速，电流控制扭矩
	int16_t Max_iout_s = 30000;
<<<<<<< HEAD
	
	int16_t speed[4];
	int16_t output[4];
	
	extern motor_info motor[4];
=======
//	float angle[4];
//	float speed[4];
//	float output[4];
	motor_info motor[4];

>>>>>>> 8c860dbd1c0c05f15a659d56d19cf6c4b0ed15fd
	for(int i=0;i<4;i++){
		pid_init(&PID_speed[i],PID_s[0],PID_s[1],PID_s[2]);
		pid_init(&PID_angle[i],PID_a[0],PID_a[1],PID_a[2]);
	}
	
  for(;;)
  {
		motor[0].speed = -1;
		for(int i=0;i<4;i++){
<<<<<<< HEAD
			speed[i] = pid_cal_a(&PID_angle[i],motor[i].angle,set_angle,Max_out_a,Max_iout_a);
			output[i] = pid_cal_s(&PID_speed[i],motor[i].speed,speed[i],Max_out_s,Max_iout_s);
		}
=======
//			pid_init(&PID_angle[i],Kp,Ki,Kd);
//			speed[i] = pid_cal(&PID_angle[i],motor[i].angle,set_angle,Max_out_a,Max_iout_a);
//			pid_init(&PID_speed[i],Kp,Ki,Kd);
//			output[i] = pid_cal(&PID_speed[i],motor[i].speed,speed[i],Max_out_s,Max_iout_s);
//			output[i] = pid_cal_s(&PID_speed[i],motor[i].speed,set,Max_out_s,Max_iout_s);
//			speed = (int)(motor[0].speed);
//			output[i] = 100*(set - motor[0].speed);
			
//			if(motor[i].speed > 30000){
//				output[i] = pid_cal_s(&PID_speed[i],(motor[i].speed - 0xffff),set,Max_out_s,Max_iout_s);
//			}
//			else{
				output[i] = pid_cal_s(&PID_speed[i],motor[i].speed,set,Max_out_s,Max_iout_s);
			}
//		}
>>>>>>> 8c860dbd1c0c05f15a659d56d19cf6c4b0ed15fd
		can_cmd_send(output[0],output[1],output[2],output[3]);
    osDelay(10);
  }
}
