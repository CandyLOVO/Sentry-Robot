#include "testTask.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "user_pid.h"
#include "rc_potocal.h"

pidTypeDef PID_6020;
int32_t motor_speed[4];

extern motor_info motor[4];
extern RC_ctrl_t rc_ctrl;

void TestTask(void const * argument)
{
	//点亮达妙开发板上LED
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	
	//6020 PID参数设置
  pid_init(&PID_6020,1,0,0);
	
	//达妙转五个电机
  for(;;)
  {
		for(int i=0;i<4;i++){
			motor_speed[i] = pid_cal_s(&PID_6020,motor[i].speed,(rc_ctrl.rc.ch[0]*8),30000,30000);
		}
		//遥控器控制电机转动
		can_send_6020(motor_speed[0],motor_speed[1],motor_speed[2],motor_speed[3]); //右拨杆左右 转四个6020电机
		can_send_9025(rc_ctrl.rc.ch[1]*80); //右拨杆上下 转MF9025
    osDelay(1);
  }
}
