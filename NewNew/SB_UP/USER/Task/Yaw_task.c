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
