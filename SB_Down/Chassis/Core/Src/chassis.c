#include "freertos.h"
#include "can.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "pid.h"
int m = 0;
void Chassis(void const * argument)
{
  for(;;)
  {
		pidTypeDef PID[4];
		float Kp = 30;
		float Ki = 0.5;
		float Kd = 10;
		float set = 1365;
		float Max_out = 2000;
		float Max_iout = 2000;
		float angle[4];
		float speed[4];
		motor_info motor[4];
		for(int i=0;i<4;i++){
			pid_init(&PID[i],Kp,Ki,Kd);
			speed[i] = pid_cal(&PID[i],motor[i].angle,set,Max_out,Max_iout);
		}
		can_cmd_send(speed[0],speed[1],speed[2],speed[3]);
		m++;
		HAL_Delay(10);
    osDelay(1);
  }
}
