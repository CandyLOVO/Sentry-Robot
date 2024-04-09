#include "Chassis_Task.h"
#include "cmsis_os.h"

void Chassis_Task(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}
