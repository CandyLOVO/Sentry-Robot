#include "main.h"
#include "struct_typedef.h"

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
	
	typedef struct //In order for the IMU not to make mistakes.
	{
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;
    fp32 max_out;  
    fp32 max_iout; 
    fp32 set;
    fp32 fdb;
    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  
    fp32 error[3]; 
	} pid_struct_t;
	
	extern fp32 pid_calc(pid_struct_t *pid, fp32 ref, fp32 set); //In order for the IMU not to make mistakes.
#endif