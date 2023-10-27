#include "freertos.h"
#include "can.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "pid.h"
#include "rc_potocal.h"
#include "handle_value.h"

extern motor_info motor[4];
extern RC_ctrl_t rc_ctrl;
//#define start 3045
int16_t set_calculate(int16_t x,int16_t y);
int16_t set;
int16_t get;

void Chassis(void const * argument)
{
	uint16_t initial_angle[4];
	for(int i=0;i<4;i++)
	{
		initial_angle[i] = motor[i].angle; //读取电机初始角度
	}
	pidTypeDef PID_angle[4];
	pidTypeDef PID_speed[4];
	float PID_s[3] = {1,0.01,0};
	float PID_a[3] = {1,0,0.01};
	int16_t Max_out_a = 8191;
	int16_t Max_iout_a = 8191;
	
	int16_t Max_out_s = 30000; //电压控制转速，电流控制扭矩
	int16_t Max_iout_s = 30000;
	
	int speed[4];
	int output[4];
	
	for(int i=0;i<4;i++){
		pid_init(&PID_speed[i],PID_s[0],PID_s[1],PID_s[2]);
		pid_init(&PID_angle[i],PID_a[0],PID_a[1],PID_a[2]);
	}
	
//	int16_t set = 1365; //60度
	
  for(;;)
  {
		set = remote_value(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]); //遥控器当前角度——set 将遥控器希望转到的角度投影
		
		for(int i=0;i<4;i++){
			get = motor_value(initial_angle[i], motor[i].angle); //电机角度——get 将电机当前角度投影
			speed[i] = pid_cal_a(&PID_angle[i],get,set,Max_out_a,Max_iout_a); //还需要再看一下
			output[i] = pid_cal_s(&PID_speed[i],motor[i].speed,speed[i],Max_out_s,Max_iout_s);
		}
		can_cmd_send(output[0],output[1],output[2],output[3]);
    osDelay(10);
  }
}

int16_t set_calculate(int16_t x,int16_t y)
{
//	Atan(x/y);
}