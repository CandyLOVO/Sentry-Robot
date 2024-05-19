#include "Yaw_Task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "MG5010_control.h"
#include "rc_potocal.h"
#include "pid_user.h"
#include "Exchange_Task.h"
#include "judge.h"

pidTypeDef pid_5010_s;
pidTypeDef pid_5010_a;
uint8_t can_send_data_5010[8];
int32_t initial_angle; //5010【面向底盘正方向】的初始编码值
float yaw_angle; //大yaw5010当前角度（0~+-180）
float target_angle_5010;
float target_speed_5010;
float output_5010;
uint8_t heart_direction[4]; //受击打装甲板
float last_target_angle_5010; //上一帧目标值
uint16_t time_delay = 0; //锁住计时
uint8_t flag_suo = 0; //瞄准后云台锁住
float tar;

extern RC_ctrl_t rc_ctrl;
extern motor_5010_info motor_5010;
extern double yaw12; //云台陀螺仪yaw值
extern float yaw; //视觉传来的目标yaw值
extern float yaw_gyro; //云台陀螺仪yaw角速度值
extern uint8_t L_tracking;
extern uint8_t R_tracking;
extern uint8_t M_tracking;
extern int8_t flag;
extern TIM_HandleTypeDef htim4;
extern Sentry_t Sentry;
extern Rx_naving Rx_nav;

void Yaw_task(void const * argument)
{
	MG5010_init(); //开启5010电机
	yaw_init(); //初始化5010电机【需要校准大yaw正方向编码值】
	osDelay(3000);
	
  for(;;)
  {
		if(flag == 1)
		{
		yaw_angle = -motor_value(initial_angle, motor_5010.angle, 65535); //将5010编码值转化为0~+-180【面向两个头，向左转为-，向右转为+】
		
		//遥控器控制模式，左->中间，右->中间            测试底盘跟随云台模式，左->最上，右->中间
		if((rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==3) || (rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==1))
		{
			yaw_control();
			if(Rx_nav.poing == 1) //导航发来的上坡指令
			{
				tar = Rx_nav.yaw_target; //将大yaw转向坡的方向
			}
		}
		
		//导航上场模式，左->最下，右->最下
		if(rc_ctrl.rc.s[0]==2 && rc_ctrl.rc.s[1]==2)
		{
			if(Rx_nav.poing == 1) //导航发来的上坡指令
			{
				target_angle_5010 = Rx_nav.yaw_target; //将大yaw转向坡的方向
//				if(target_angle_5010 > 180) //将叠加坡的差角后的角度转化为0~+-180
//				{
//					target_angle_5010 -= 360;
//				}
//				else if(target_angle_5010 < -180)
//				{
//					target_angle_5010 += 360;
//				}
			}
			else //进行自瞄
			{
				//四个摄像头都没有识别到
				if(L_tracking==0 && R_tracking==0 && M_tracking==0)
				{
					//装甲板没有收到击打
					if(Sentry.armor_id == 0)
					{
						//大yaw瞄准延时
						if((flag_suo == 1)&&(time_delay <= 1000)) //上一个状态为锁住，且在1000ms内：
						{
							target_angle_5010 = last_target_angle_5010; //目标角度为锁住时的角度
						}
						//正常巡航
						else
						{
							yaw_finding(); //大yaw巡航
							flag_suo = 2; //未锁住标志位
						}
					}
					//装甲板受击打
					else
					{
						target_angle_5010 = yaw12 + heart_direction[Sentry.armor_id-1]; //装甲板受击打方位
					}
				}
				//至少有一个摄像头识别到
				else if(L_tracking==1 || R_tracking==1 || M_tracking==1)
				{
					time_delay = 0; //初始化延时计数值
					flag_suo = 1; //锁住标志位
					yaw_suoing(); //大yaw响应
					last_target_angle_5010 = target_angle_5010; //保存该次锁住的目标值
					HAL_TIM_Base_Start_IT(&htim4); //启动定时器TIM4
				}
			}
		}
		
		//PID控制
		target_speed_5010 = pid_I_control(&pid_5010_a, yaw12, target_angle_5010);
		output_5010 = pid_cal_s(&pid_5010_s, (9.55f*yaw_gyro), target_speed_5010);
		//CAN2数据发送
		speed_control_send(output_5010);
		can_cmd_send_5010(can_send_data_5010);
		}
    osDelay(1);
  }
}

/*********************************************************函数实现*********************************************************/
void yaw_init(void)
{
	//大yaw5010数值初始化
	initial_angle = 36528; //头朝向底盘正方向时的编码值
	target_angle_5010 = 0;
	target_speed_5010 = 0;
	pid_init(&pid_5010_s,20000,0,0,700000,700000); //PID初始化 PI
	pid_init(&pid_5010_a,3,0,0,700000,700000); //PD
//	pid_init(&pid_5010_s,7000,2,0,700000,700000); //PID??? PI
//	pid_init(&pid_5010_a,4,0,23,700000,700000); //PD
	
	heart_direction[0] = 0;
	heart_direction[1] = 90;
	heart_direction[2] = 180;
	heart_direction[3] = -90;
}

void yaw_control(void)
{
	target_angle_5010 += rc_ctrl.rc.ch[2]*0.1/660;
	if(target_angle_5010 > 180)
	{
		target_angle_5010 -= 360;
	}
	else if(target_angle_5010 < -180)
	{
		target_angle_5010 += 360;
	}
}

void yaw_finding(void)
{
	//大yaw巡航
	target_angle_5010 += 0.03;
	if(target_angle_5010 > 180)
	{
		target_angle_5010 -= 360;
	}
	else if(target_angle_5010 < -180)
	{
		target_angle_5010 += 360;
	}
}

void yaw_suoing(void)
{
	//至少有一个摄像头瞄准时
	if(L_tracking==1 && R_tracking==1) //两个头都锁住
	{
		if((yaw-yaw12)<20 && (yaw-yaw12)>-20) //转动角度小于阈值
		{
			target_angle_5010 = target_angle_5010;
		}
		else
		{
			target_angle_5010 = yaw;
		}
	}
	else if(L_tracking==0 || R_tracking==0) //有一个头没有锁住
	{
		target_angle_5010 = yaw;
	}
	else //中间的头锁住
	{
		target_angle_5010 = yaw;
	}
}

float motor_value(int32_t k, int32_t n, int32_t max)
{
	//将MG5010电机(通用版)的角度以初始角度为0，映射到0~+-180度 (k:设定的初始角度“0” ； n:想要映射的角度) 得到的是倒装电机转化的0~+-180
	//第3个参数是编码器最大值
	int32_t middle = (max+1)/2;
	float output;
	if(k>=0 && k<middle){
		if(n>=0 && n<(k+middle)){
			n = k - n;
			output = (float)n * 360.f / (float)max;
			return output;
		}
		else if(n>=(k+middle) && n<max){
			n = max - n + k;
			output = (float)n * 360.f / (float)max;
			return output;
		}
	}
	
	else if(k>=middle && k<max){
		if(n>=0 && n<(k-middle)){
			n = -max + k - n;
			output = (float)n * 360.f / (float)max;
			return output;
		}
		else if(n>=(k-middle) && n<max){
			n = k - n;
			output = (float)n * 360.f / (float)max;
			return output;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim4))
  {
		time_delay++;
	}
}
/**************************************************************************************************************************/
