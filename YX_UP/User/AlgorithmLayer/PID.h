#ifndef __PID_H
#define __PID_H
#include "main.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))		//越界则赋边界值


typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value  
  float err[2];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;


void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
							
float pid_calc(pid_struct_t *pid, float ref, float fdb);	//速度环PID计算,ref是速度目标值，fdb是速度返回值
float pid_calc_sita(pid_struct_t *pid,float ref, float fdb);	//位置环PID计算，单位是角度，ref是角度目标值，fdb是角度返回值

//================================================PID结构体声明================================================//
extern pid_struct_t motor_pid[7];	
extern pid_struct_t motor_pid_sita[7];
extern pid_struct_t motor_pid_can_2[7];	
extern pid_struct_t motor_pid_sita_can_2[7];
							
#endif
							