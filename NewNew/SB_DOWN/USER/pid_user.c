#include "pid_user.h"

float limit_max(float value,float Max_out)
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

void pid_init(pidTypeDef *PID,float p,float i,float d,int32_t Max_out,int32_t Max_iout)
{
	PID->error[0] = 0;
	PID->error[1] = 0;
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

float pid_cal_a(pidTypeDef *PID,float get,float set) //set is target
{
	PID->get = get;
	PID->set = set;

	PID->error[0] = PID->error[1];
	PID->error[1] = PID->set - PID->get;
	
	//电机按照最短路径转回去
	if(PID->error[1] > 180){
		PID->error[1] = PID->error[1] - 360;
	}
	else if(PID->error[1] < -180){
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

//积分分离
float pid_I_control(pidTypeDef *PID,float get,float set) //set is target
{
	float index_pid = 0;
	PID->get = get;
	PID->set = set;

	PID->error[0] = PID->error[1];
	PID->error[1] = PID->set - PID->get;
	
	//电机按照最短路径转回去
	if(PID->error[1] > 180){
		PID->error[1] = PID->error[1] - 360;
	}
	else if(PID->error[1] < -180){
		PID->error[1] = PID->error[1] + 360;
	}
	else{
		PID->error[1] = PID->error[1];
	}
	
	if(fabs(PID->error[1])>0.1) //误差值小于阈值
	{
		index_pid = 0; //不加积分项
	}
	else
	{
		index_pid = 1; //加入积分项
		PID->integral += PID->error[1];
	}
	
	PID->pout = PID->Kp * PID->error[1];
	PID->iout = index_pid * PID->Ki * PID->integral;
	PID->iout = limit_max(PID->iout,PID->Max_iout);
	PID->dout = PID->Kd * (PID->error[1] - PID->error[0]);
	PID->out = PID->pout + PID->iout + PID->dout;
	PID->out = limit_max(PID->out,PID->Max_out);
	return PID->out;
}
