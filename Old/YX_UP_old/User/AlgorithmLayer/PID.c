#include "PID.h"

//================================================PID结构体定义================================================//
pid_struct_t motor_pid[7];	
pid_struct_t motor_pid_sita[7];
pid_struct_t motor_pid_can_2[7];	
pid_struct_t motor_pid_sita_can_2[7];

//================================================PID初始化================================================//
//	Input parameter: 
//	struct pid 
//	p
//	i
//	d
//	积分限幅最大值i_max
//	pid结果输出最大值out_max

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

//================================================PID速度环================================================//
//	Input parameter:
//	struct pid
//	target 目标速度值---->对应Pid结构体参数中的ref
//	respend 当前速度值(电机返回值)---->对应Pid结构体参数中的fdb

float pid_calc(pid_struct_t *pid, float target, float respond)
{
  pid->ref = target;
  pid->fdb = respond;
  pid->err[1] = pid->err[0];//err[1]是上一次计算出来的差值
  pid->err[0] = pid->ref - pid->fdb;//err[0]是这一次的预期速度和实际速度的差值,这两个值是可以是负数的
  
  pid->p_out  = pid->kp * pid->err[0];//40 3 0是标准值，把这个加到watch1里面
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//防止越界
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);//防止越界
  return pid->output;
}

//================================================PID角度环================================================//
//	Input parameter:
//	struct pid
//	target 目标角度值---->对应Pid结构体参数中的ref
//	respend 当前角度值(电机返回值)---->对应Pid结构体参数中的fdb
//	注意：单位是角度(非弧度)

float pid_calc_sita(pid_struct_t *pid,float target, float respond) //位置环PID计算，单位是角度，ref是角度目标值，fdb是角度返回值
{
  pid->ref = target;
  pid->fdb = respond;
	pid->err[1] = pid->err[0];
	
	float err = 0;
	err = target - respond;
	//越界处理
	if(err > 180)
	{
		err -= 360;
	}
	else if(err < -180)
	{
		err += 360;
	}
  pid->err[0] = err;
	
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
	
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//防止i值叠加越界
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);//防止总体越界
  return pid->output;
}
