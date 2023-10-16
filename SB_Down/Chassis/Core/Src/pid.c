#include "pid.h"

float limit_max(float value,float Max_out)
{
	if(value > Max_out){
		value = Max_out;
	}
	if(value < -Max_out){
		value = -Max_out;
	}
	return value;
}

void pid_init(pidTypeDef *PID,float p,float i,float d)
{
	PID->error[0] = 0;
	PID->error[1] = 0;
	PID->Kp = p;
	PID->Ki = i;
	PID->Kd = d;
}

float pid_cal(pidTypeDef *PID,float get,float set,float Max_out,float Max_iout)
{
	PID->get = get;
	PID->set = set;
	PID->Max_out = Max_out;
	PID->Max_iout = Max_iout;
	PID->error[0] = PID->error[1];
	PID->error[1] = PID->set - PID->get;
	PID->pout = PID->Kp * PID->error[1];
	PID->iout += PID->Ki * PID->error[1];
	PID->iout = limit_max(PID->iout,Max_iout);
	PID->dout = PID->Kd * (PID->error[1] - PID->error[0]);
	PID->out = PID->pout + PID->iout + PID->dout;
	PID->out = limit_max(PID->out,PID->Max_out);
	return PID->out;
}