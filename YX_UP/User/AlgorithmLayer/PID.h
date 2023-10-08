#ifndef __PID_H
#define __PID_H
#include "main.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))		//越界则赋边界值

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
							
float pid_calc(pid_struct_t *pid, float ref, float fdb);	//速度环PID计算,ref是速度目标值，fdb是速度返回值
float pid_calc_sita(pid_struct_t *pid,float ref, float fdb);	//位置环PID计算，单位是角度，ref是角度目标值，fdb是角度返回值
					
#endif
							