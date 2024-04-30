#include "Pitch_task.h"
#include "cmsis_os.h"
#include "pid_user.h"
#include "Yaw_task.h"
#include "can_user.h"
#include "Exchange_task.h"
#include "fdcan.h"
#include "imu_temp_ctrl.h"

//**************************************************变量定义**************************************************//
pidTypeDef pid_pitch_s_L;
pidTypeDef pid_pitch_a_L;
pidTypeDef pid_pitch_s_R;
pidTypeDef pid_pitch_a_R;
int32_t initial_pitch_L;
int32_t initial_pitch_R;
float pitch_angle_L;
float pitch_angle_R;
float target_pitch_a_L;
float target_pitch_s_L;
float target_pitch_a_R;
float target_pitch_s_R;
int16_t pitch_output_L;
int16_t pitch_output_R;
uint8_t nod_flag_L = 0; //判断是否需要反转
uint8_t nod_flag_R = 0;

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;
extern receive_vision Rx_vision;
extern uint8_t gimbal_control_6020[8];
extern int8_t flag;
//***********************************************************************************************************//

//**************************************************任务实现**************************************************//

//黑头（左）ID为4，白头（右）ID为3
void Pitch_Task(void const * argument)
{
	Pitch_init();
//	osDelay(3000);
  for(;;)
  {
		//判断陀螺仪是否温补结束
		if(flag == 1)
		{
		//初始化映射pitch的角度
		pitch_angle_L = -motor_value(initial_pitch_L, motor[3].angle); //【要给负值！！】
		pitch_angle_R = motor_value(initial_pitch_R, motor[2].angle); //pitch处的6020电机镜像装配，需要在取反的基础上再取反
		
		//遥控器控pitch模式，左->中，右->上
		if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==1)
		{
			target_pitch_a_L += rc_ctrl.rc.ch[3] * 0.1/660;
			target_pitch_a_R += rc_ctrl.rc.ch[1] * 0.1/660;
			pitch_control_L(12, -24);
			pitch_control_R(12, -24);
		}
		
		//自瞄模式，左->下，右->下
		if(rc_ctrl.rc.s[1]==2 && rc_ctrl.rc.s[0]==2)
		{
			//都没有识别到目标，开始巡航
			if(Rx_vision.L_tracking==0 && Rx_vision.R_tracking==0 && Rx_vision.M_tracking==0)
			{
				if(nod_flag_L == 0)
				{
					nod_flag_L = 1;
				}
				if(nod_flag_R == 0)
				{
					nod_flag_R = 1;
				}
				
				pitch_finding(12, -24);
			}
			
			//左头识别到目标
			if(Rx_vision.L_tracking == 1)
			{
				//左头停止巡航
				nod_flag_L = 0;
				target_pitch_a_L = Rx_vision.L_pitch;
				pitch_control_L(12, -24);
			}
			if(Rx_vision.R_tracking == 1)
			{
				//右头停止巡航
				nod_flag_R = 0;
				target_pitch_a_R = Rx_vision.R_pitch;
				pitch_control_R(12, -24);
			}
			if(Rx_vision.M_tracking == 1)
			{
				nod_flag_L = 0;
				target_pitch_a_L = Rx_vision.L_pitch;
				pitch_control_L(12, -24);
				nod_flag_R = 0;
				target_pitch_a_R = Rx_vision.R_pitch;
				pitch_control_R(12, -24);
			}
		}
		
		target_pitch_s_L = pid_cal_a(&pid_pitch_a_L, pitch_angle_L, target_pitch_a_L);
		pitch_output_L = pid_cal_s(&pid_pitch_s_L, motor[3].speed, target_pitch_s_L);
		target_pitch_s_R = -pid_cal_a(&pid_pitch_a_R, pitch_angle_R, target_pitch_a_R);
		pitch_output_R = pid_cal_s(&pid_pitch_s_R, motor[2].speed, target_pitch_s_R);
		
//		gimbal_control_6020[4] = (pitch_output_L>>8)&0xff;
//		gimbal_control_6020[5] = pitch_output_L&0xff;
//		gimbal_control_6020[6] = (pitch_output_R>>8)&0xff;
//		gimbal_control_6020[7] = pitch_output_R&0xff;
		gimbal_control_6020[6] = (pitch_output_L>>8)&0xff;
		gimbal_control_6020[7] = pitch_output_L&0xff;
		gimbal_control_6020[4] = (pitch_output_R>>8)&0xff;
		gimbal_control_6020[5] = pitch_output_R&0xff;
		canx_send_data(&hfdcan1, 0x1FF, gimbal_control_6020, 8);
    osDelay(1);
		}
  }
}
//***********************************************************************************************************//

//**************************************************代码实现**************************************************//
static void Pitch_init(void)
{
	initial_pitch_L = 8171;
	initial_pitch_R = 90;
	
	//PID初始化
	pid_init(&pid_pitch_s_L,170,0.3,0,30000,30000); //PID初始化 PI
	pid_init(&pid_pitch_a_L,20,0,30,30000,30000); //PD
	
	pid_init(&pid_pitch_s_R,1,0,0,30000,30000); //PID初始化
	pid_init(&pid_pitch_a_R,1,0,0,30000,30000);
}

void pitch_control_L(int16_t max_angle, int16_t min_angle)
{
	if(target_pitch_a_L > 180)
	{
		target_pitch_a_L -= 360;
	}
	else if(target_pitch_a_L < -180)
	{
		target_pitch_a_L += 360;
	}
	
	if(target_pitch_a_L > max_angle)
	{
		target_pitch_a_L = max_angle;
	}
	else if(target_pitch_a_L < min_angle)
	{
		target_pitch_a_L = min_angle;
	}
}

void pitch_control_R(int16_t max_angle, int16_t min_angle)
{
	if(target_pitch_a_R > 180)
	{
		target_pitch_a_R -= 360;
	}
	else if(target_pitch_a_R < -180)
	{
		target_pitch_a_R += 360;
	}
	
	if(target_pitch_a_R > max_angle)
	{
		target_pitch_a_R = max_angle;
	}
	else if(target_pitch_a_R < min_angle)
	{
		target_pitch_a_R = min_angle;
	}
}

void pitch_finding(int16_t max_angle, int16_t min_angle)
{
	if(nod_flag_L == 1)
	{
		target_pitch_a_L -= 0.05;
		if(target_pitch_a_L < min_angle)
		{
			nod_flag_L = 2;
		}
	}
	else if(nod_flag_L == 2)
	{
		target_pitch_a_L += 0.05;
		if(target_pitch_a_L > max_angle)
		{
			nod_flag_L = 1;
		}
	}

	if(nod_flag_R == 1)
	{
		target_pitch_a_R -= 0.05;
		if(target_pitch_a_R < max_angle)
		{
			nod_flag_R = 2;
		}
	}
	else if(nod_flag_R == 2)
	{
		target_pitch_a_R += 0.05;
		if(target_pitch_a_R > max_angle)
		{
			nod_flag_R = 1;
		}
	}
}
//***********************************************************************************************************//
