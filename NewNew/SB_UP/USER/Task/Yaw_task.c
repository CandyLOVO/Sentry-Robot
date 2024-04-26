#include "Yaw_task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "fdcan.h"
#include "main.h"
#include "pid_user.h"

pidTypeDef pid_yaw_s;
pidTypeDef pid_yaw_a;
uint8_t can_send_data[8];
int32_t initial_angle_L;
int32_t initial_angle_R;
float yaw_angle_L;
float yaw_angle_R;
int16_t target_yaw_a_L;
int16_t target_yaw_s_L;
int16_t target_yaw_a_R;
int16_t target_yaw_s_R;

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;

//	两个头yaw用的FDCAN1,ID是1和2 【左右ID有待测试，只需要更改motor[]中的序号】
void Yaw_Task(void const * argument)
{
	Yaw_init();
	osDelay(3000);
  for(;;)
  {
		//根据yaw的初始位置映射yaw的角度
		yaw_angle_L = motor_value(initial_angle_L, motor[0].angle);
		yaw_angle_R = motor_value(initial_angle_R, motor[1].angle);
		
		//遥控器控小yaw模式，左->中，右->上
		if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==1)
		{
			yaw_control_L(-170,-10);
			yaw_control_R(0,0);
		}
		osDelay(1);
  }
}

//================================================YAW轴PID参数和目标IMU初始化================================================//
static void Yaw_init(void)
{
	initial_angle_L = 8192;
	initial_angle_R = 8192;
	target_yaw_a_L = 0;
	target_yaw_s_L = 0;
	target_yaw_a_R = 0;
	target_yaw_s_R = 0;
	pid_init(&pid_yaw_s,10,0,0,30000,30000); //PID初始化
	pid_init(&pid_yaw_a,1,0,0,30000,30000);
}

void yaw_control_L(int16_t max_angle, int16_t min_angle)
{
	int16_t error;
	
	//遥控器控制
	target_yaw_a_L += rc_ctrl.rc.ch[2] * 30/660;
	
	//越界处理
	if(target_yaw_a_L>180)
	{
		target_yaw_a_L -= 360;
	}
	else if(target_yaw_a_L<-180)
	{
		target_yaw_a_L += 360;
	}
	
	//软件限位
	if((target_yaw_a_L>min_angle) && (target_yaw_a_L<max_angle))
	{
		if((yaw_angle_L>min_angle) && (yaw_angle_L<max_angle))
		{
			if(motor[0].speed>0)
			{
				target_yaw_a_L = min_angle;
			}
			else if(motor[0].speed<0)
			{
				target_yaw_a_L = max_angle;
			}
		}
	}
	
}

void yaw_control_R(int16_t max_angle, int16_t min_angle)
{
	//遥控器控制
	target_yaw_a_R += rc_ctrl.rc.ch[0] * 30/660;
	
	//越界处理
	if(target_yaw_a_R>180)
	{
		target_yaw_a_R -= 360;
	}
	else if(target_yaw_a_R<-180)
	{
		target_yaw_a_R += 360;
	}
	
	//软件限位
	if(target_yaw_a_R>max_angle)
	{
		target_yaw_a_R = max_angle;
	}
	else if(target_yaw_a_R<min_angle)
	{
		target_yaw_a_R = min_angle;
	}
}

//将6020电机的角度以初始角度为0，映射到0~+-180度 (k:设定的初始角度“0” ； n:想要映射的角度)
//该解算为倒装6020的映射，正装6020需要将输出结果加“-”
float motor_value(int16_t k, int16_t n)
{
	float ans;
	if(k>=0 && k<4096){
		if(n>=0 && n<(k+4096)){
			n = k - n;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
		else if(n>=(k+4096) && n<8191){
			n = 8192 - n + k;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
	}
	
	else if(k>=4096 && k<8192){
		if(n>=0 && n<(k-4096)){
			n = -8192 + k - n;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
		else if(n>=(k-4096) && n<8191){
			n = k - n;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
	}
}
