#ifndef __PID_USER_H__
#define __PID_USER_H__

#include "main.h"

typedef struct
{
	float Kp, Ki, Kd;
	int32_t error[2];
	int32_t get, set;
	int32_t pout, iout, dout, out;
	int32_t integral;
	int32_t Max_out;
	int32_t Max_iout;
}pidTypeDef;
	
extern int32_t limit_max(int32_t value,int32_t Max_out);
extern void pid_init(pidTypeDef *PID,float p,float i,float d,int32_t Max_out,int32_t Max_iout);
extern int32_t pid_cal_s(pidTypeDef *PID,float get,float set);
extern int32_t pid_cal_a(pidTypeDef *PID,float get,float set);
extern int32_t pid_I_control(pidTypeDef *PID,float get,float set);

#endif
