#ifndef __PID_H
#define __PID_H
#include "main.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))		//Խ���򸳱߽�ֵ

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
							
float pid_calc(pid_struct_t *pid, float ref, float fdb);	//�ٶȻ�PID����,ref���ٶ�Ŀ��ֵ��fdb���ٶȷ���ֵ
float pid_calc_sita(pid_struct_t *pid,float ref, float fdb);	//λ�û�PID���㣬��λ�ǽǶȣ�ref�ǽǶ�Ŀ��ֵ��fdb�ǽǶȷ���ֵ
					
#endif
							