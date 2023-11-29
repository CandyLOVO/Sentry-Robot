#include "user_pid.h"

int16_t limit_max(int16_t value,int16_t Max_out)
{
	if(value > Max_out){
		value = Max_out;
	}
	else if(value < -Max_out){
		value = -Max_out;
	}
	else{
		value = value;
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

int16_t pid_cal_s(pidTypeDef *PID,int16_t get,int16_t set,int16_t Max_out,int16_t Max_iout)
{
	PID->get = get;
	PID->set = set;
	PID->Max_out = Max_out;
	PID->Max_iout = Max_iout;
	PID->error[0] = PID->error[1];
	PID->error[1] =  PID->set - PID->get;
	
	PID->pout = PID->Kp * PID->error[1];
	PID->iout += PID->Ki * PID->error[1];
	PID->iout = limit_max(PID->iout,PID->Max_iout);
	PID->dout = PID->Kd * (PID->error[1] - PID->error[0]);
	PID->out = PID->pout + PID->iout + PID->dout;
	PID->out = limit_max(PID->out,PID->Max_out);
	return PID->out;
}

int16_t pid_cal_a(pidTypeDef *PID,float get,float set,int16_t Max_out,int16_t Max_iout)
{
	PID->get = get;
	PID->set = set;
	PID->Max_out = Max_out;
	PID->Max_iout = Max_iout;
	PID->error[0] = PID->error[1];
	PID->error[1] = PID->set - PID->get;
//	if((PID->set - PID->get) > 4096){
//		PID->get = PID->get + 8192;
//	}
//	else if((PID->set - PID->get) < -4096){
//		PID->get = PID->get - 8192;
//	}
//	else{
//		PID->get = PID->get;
//	}
	
	if(PID->error[1] > 180){
		PID->error[1] = PID->error[1] - 360;
	}
	else if((PID->set - PID->get) < -180){
		PID->error[1] = PID->error[1] + 360;
	}
	else{
		PID->error[1] = PID->error[1];
	}
	
	PID->pout = PID->Kp * PID->error[1];
	PID->iout += PID->Ki * PID->error[1];
	PID->iout = limit_max(PID->iout,PID->Max_iout);
	PID->dout = PID->Kd * (PID->error[1] - PID->error[0]);
	PID->out = PID->pout + PID->iout + PID->dout;
	PID->out = limit_max(PID->out,PID->Max_out);
	return PID->out;
}