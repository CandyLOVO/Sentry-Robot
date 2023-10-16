#include "main.h"

#ifndef __PID_H__
#define __PID_H__
	typedef struct
	{
		float Kp, Ki, Kd;
		float error[2];
		float ierror;
		float get, set;
		float pout, iout, dout, out;
		float Max_out;
		float Max_iout;
	}pidTypeDef;
	
	float limit_max(float value,float Max_out);
	void pid_init(pidTypeDef *PID,float p,float i,float d);
	float pid_cal(pidTypeDef *PID,float get,float set,float Max_out,float Max_iout);
#endif