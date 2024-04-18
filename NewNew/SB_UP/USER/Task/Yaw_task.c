#include "Yaw_task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "fdcan.h"
#include "main.h"
#include "MG5010_control.h"

uint8_t can_send_data[8];
uint8_t can_send_data_5010[8];
extern motor_info motor[8];

void Yaw_Task(void const * argument)
{
	start_5010();
  for(;;)
  {
		
		speed_control_send(100000);
		canx_send_data(&hfdcan1, 0x141, can_send_data_5010, 8);
		osDelay(1);
  }
}

//================================================YAW轴PID参数和目标IMU初始化================================================//
static void Yaw_init()
{
}
