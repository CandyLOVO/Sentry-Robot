#ifndef __PID_USER_H__
#define __PID_USER_H__

#include "main.h"

typedef struct
{
	float Kp, Ki, Kd;
	float error0;
	float error1;
	float get, set;
	float pout, iout, dout, out;
	float Max_out;
	float Max_iout;
}pidTypeDef;
	
extern float limit_max(float value,float Max_out);
extern void pid_init(pidTypeDef *PID,float p,float i,float d,float Max_out,float Max_iout);
extern float pid_cal_s(pidTypeDef *PID,float get,float set);
extern float pid_cal_a(pidTypeDef *PID,float get,float set);
extern float pid_cal_yaw_a(pidTypeDef *PID,float get,float set,uint8_t warning_flag,float up_limit,float low_limit);
extern float pid_cal_yaw_a_for_nan(pidTypeDef *PID,float get,float set,uint8_t warning_flag,float up_limit,float low_limit);

#endif
