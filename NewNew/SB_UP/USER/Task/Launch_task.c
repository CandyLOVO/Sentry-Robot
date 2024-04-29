#include "Launch_task.h"
#include "cmsis_os.h"
#include "can_user.h"



extern motor_info motor_friction[8];

void Launch_Task(void const * argument)
{
  for(;;)
  {
		
    osDelay(1);
	}
}
