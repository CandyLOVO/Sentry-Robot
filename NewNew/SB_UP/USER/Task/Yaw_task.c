#include "Yaw_task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "fdcan.h"
#include "main.h"

uint8_t can_send_data[8];
extern motor_info motor[8];

void Yaw_Task(void const * argument)
{
	osDelay(3000);
  for(;;)
  {
		osDelay(1);
  }
}

//================================================YAW轴PID参数和目标IMU初始化================================================//
static void Yaw_init()
{
}
