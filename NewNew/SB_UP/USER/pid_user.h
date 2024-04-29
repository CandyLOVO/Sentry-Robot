#ifndef __PID_USER_H__
#define __PID_USER_H__

#include "main.h"

typedef struct
{
	float Kp, Ki, Kd;
	int16_t error[2];
	int16_t get, set;
	int32_t pout, iout, dout, out;
	int16_t Max_out;
	int16_t Max_iout;
}pidTypeDef;
	
extern float limit_max(float value,float Max_out);
extern void pid_init(pidTypeDef *PID,float p,float i,float d,int16_t Max_out,int16_t Max_iout);
extern float pid_cal_s(pidTypeDef *PID,float get,float set);
extern float pid_cal_a(pidTypeDef *PID,float get,float set);
extern float pid_cal_yaw_a(pidTypeDef *PID,float get,float set,uint8_t warning_flag);

#endif
