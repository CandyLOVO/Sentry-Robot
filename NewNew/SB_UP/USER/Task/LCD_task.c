#include "LCD_task.h"
#include "cmsis_os.h"
#include "lcd.h"
#include "pic.h"

extern uint16_t adc_val[1]; //存储ADC采集到的数据

void LCD_Task(void const * argument)
{
  for(;;)
  {
		LCD_ShowString(120, 72,(uint8_t *)"dmBot", BRRED, BLACK, 24, 0);
		LCD_ShowChinese(84, 100, (uint8_t *)"达妙科技", WHITE, BLACK, 32, 0);
		LCD_DrawLine(10, 0, 10,  280,WHITE);
		LCD_DrawLine(270,0, 270, 280,WHITE);
		LCD_ShowIntNum(50, 170, adc_val[0], 5, WHITE, BLACK, 32);
		LCD_ShowPicture(180, 150, 80, 80, gImage_1);
    osDelay(1);
  }
}
