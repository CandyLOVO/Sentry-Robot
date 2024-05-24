#include "pid_user.h"
#include "Yaw_task.h"

extern int16_t up_limit_L;
extern int16_t low_limit_L;
extern int16_t up_limit_R;
extern int16_t low_limit_R;

float limit_max(float value,float Max_out)
{
	if(value >= Max_out){
		value = Max_out;
	}
	else if(value <= -Max_out){
		value = -Max_out;
	}
	else{
		value = value;
	}
	return value;
}

void pid_init(pidTypeDef *PID,float p,float i,float d,float Max_out,float Max_iout)
{
	PID->error0 = 0;
	PID->error1 = 0;
	PID->Kp = p;
	PID->Ki = i;
	PID->Kd = d;
	PID->Max_out = Max_out;
	PID->Max_iout = Max_iout;
}

float pid_cal_s(pidTypeDef *PID,float get,float set) //set is target
{
	PID->get = get;
	PID->set = set;
	
	PID->error0 = PID->error1;
	PID->error1 =  PID->set - PID->get;
	
	PID->pout = PID->Kp * PID->error1;
	PID->iout += PID->Ki * PID->error1;
	PID->iout = limit_max(PID->iout,PID->Max_iout);
	PID->dout = PID->Kd * (PID->error1 - PID->error0);
	PID->out = PID->pout + PID->iout + PID->dout;
	PID->out = limit_max(PID->out,PID->Max_out);
	return PID->out;
}

float pid_cal_a(pidTypeDef *PID,float get,float set) //set is target
{
	PID->get = get;
	PID->set = set;

	PID->error0 = PID->error1;
	PID->error1 = PID->set - PID->get;
	
	if(PID->error1 > 180){
		PID->error1 = PID->error1 - 360;
	}
	else if(PID->error1 < -180){
		PID->error1 = PID->error1 + 360;
	}
	else{
		PID->error1 = PID->error1;
	}
	
	PID->pout = PID->Kp * PID->error1;
	PID->iout += PID->Ki * PID->error1;
	PID->iout = limit_max(PID->iout,PID->Max_iout);
	PID->dout = PID->Kd * (PID->error1 - PID->error0);
	PID->out = PID->pout + PID->iout + PID->dout;
	PID->out = limit_max(PID->out,PID->Max_out);
	return PID->out;
}

float pid_cal_yaw_a(pidTypeDef *PID,float get,float set,uint8_t warning_flag,float up_limit,float low_limit) //set is target
{
	PID->get = get;
	PID->set = set;

	PID->error0 = PID->error1;
	PID->error1 = PID->set - PID->get;
	
	if(PID->error1 > 180){
		PID->error1 = PID->error1 - 360;
	}
	else if(PID->error1 < -180){
		PID->error1 = PID->error1 + 360;
	}
	else{
		PID->error1 = PID->error1;
	}
	
	if(warning_flag == 1)
	{
		if((PID->error1>=-180) && (PID->error1<=up_limit))
		{
			PID->error1 += 360;
		}
	}
	else if(warning_flag == 2)
	{
		if((PID->error1>=low_limit) && (PID->error1<=180))
		{
			PID->error1 -= 360;
		}
	}
	else
	{
		PID->error1 = PID->error1;
	}
	
	PID->pout = PID->Kp * PID->error1;
	PID->iout += PID->Ki * PID->error1;
	PID->iout = limit_max(PID->iout,PID->Max_iout);
	PID->dout = PID->Kd * (PID->error1 - PID->error0);
	PID->out = PID->pout + PID->iout + PID->dout;
	PID->out = limit_max(PID->out,PID->Max_out);
	return PID->out;
}

float pid_cal_yaw_a_for_nan(pidTypeDef *PID,float get,float set,uint8_t warning_flag,float up_limit,float low_limit) //set is target
{
	PID->get = get;
	PID->set = set;

	PID->error0 = PID->error1;
	PID->error1 = PID->set - PID->get;
	
	if(PID->error1 > 180){
		PID->error1 = PID->error1 - 360;
	}
	else if(PID->error1 < -180){
		PID->error1 = PID->error1 + 360;
	}
	else{
		PID->error1 = PID->error1;
	}
	
	if(warning_flag == 1)
	{
		if((PID->error1>=-180) && (PID->error1<=up_limit))
		{
			PID->error1 += 360;
		}
	}
	else if(warning_flag == 2)
	{
		if((PID->error1>=low_limit) && (PID->error1<=180))
		{
			PID->error1 -= 360;
		}
	}
	else
	{
		PID->error1 = PID->error1;
	}
	
	PID->pout = PID->Kp * PID->error1;
	PID->iout += 0.0 * PID->error1;
	PID->iout = limit_max(PID->iout,PID->Max_iout);
	PID->dout = PID->Kd * (PID->error1 - PID->error0);
	PID->out = PID->pout + PID->iout + PID->dout;
	PID->out = limit_max(PID->out,PID->Max_out);
	return PID->out;
}
