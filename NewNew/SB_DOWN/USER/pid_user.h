#ifndef __PID_USER_H__
#define __PID_USER_H__

#include "main.h"

typedef struct
{
	float Kp, Ki, Kd;
	float error[2];
	float get, set;
	float pout, iout, dout, out;
	int32_t integral;
	int32_t Max_out;
	int32_t Max_iout;
}pidTypeDef;
	
extern float limit_max(float value,float Max_out);
extern void pid_init(pidTypeDef *PID,float p,float i,float d,int32_t Max_out,int32_t Max_iout);
extern float pid_cal_s(pidTypeDef *PID,float get,float set);
extern float pid_cal_a(pidTypeDef *PID,float get,float set);
extern float pid_I_control(pidTypeDef *PID,float get,float set);

#endif
