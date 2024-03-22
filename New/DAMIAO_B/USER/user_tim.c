#include "user_tim.h"
float Sys_time;

void TIM2_Init()
{
	Sys_time=0;
	
HAL_TIM_Base_Start_IT(&htim2);
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance) 
	{	
	Sys_time+=0.1;
	}
}
