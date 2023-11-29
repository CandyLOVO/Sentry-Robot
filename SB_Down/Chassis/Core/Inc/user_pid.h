#include "main.h"

#ifndef __PID_H__
#define __PID_H__
	typedef struct
	{
		float Kp, Ki, Kd;
		int16_t error[2];
		int16_t get, set;
		int16_t pout, iout, dout, out;
		int16_t Max_out;
		int16_t Max_iout;
	}pidTypeDef;
	
	int16_t limit_max(int16_t value,int16_t Max_out);
	void pid_init(pidTypeDef *PID,float p,float i,float d);
	int16_t pid_cal_s(pidTypeDef *PID,int16_t get,int16_t set,int16_t Max_out,int16_t Max_iout);
	int16_t pid_cal_a(pidTypeDef *PID,float get,float set,int16_t Max_out,int16_t Max_iout);
	
#endif