#include "Pitch_task.h"
#include "cmsis_os.h"
#include "pid_user.h"
#include "Yaw_task.h"
#include "can_user.h"
#include "Exchange_task.h"
#include "fdcan.h"
#include "imu_temp_ctrl.h"

//**************************************************��������**************************************************//
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
uint8_t nod_flag_L = 0; //�ж��Ƿ���Ҫ��ת
uint8_t nod_flag_R = 0;

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;
extern receive_vision Rx_vision;
extern uint8_t gimbal_control_6020[8];
extern int8_t flag;
//***********************************************************************************************************//

//**************************************************����ʵ��**************************************************//

//��ͷ����IDΪ4����ͷ���ң�IDΪ3
void Pitch_Task(void const * argument)
{
	Pitch_init();
//	osDelay(3000);
  for(;;)
  {
		//�ж��������Ƿ��²�����
		if(flag == 1)
		{
		//��ʼ��ӳ��pitch�ĽǶ�
		pitch_angle_L = -motor_value(initial_pitch_L, motor[3].angle); //��Ҫ����ֵ������
		pitch_angle_R = motor_value(initial_pitch_R, motor[2].angle); //pitch����6020�������װ�䣬��Ҫ��ȡ���Ļ�������ȡ��
		
		//ң������pitchģʽ����->�У���->��
		if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==1)
		{
			target_pitch_a_L += rc_ctrl.rc.ch[3] * 0.1/660;
			target_pitch_a_R += rc_ctrl.rc.ch[1] * 0.1/660;
			pitch_control_L(12, -24);
			pitch_control_R(12, -24);
		}
		
		//����ģʽ����->�£���->��
		if(rc_ctrl.rc.s[1]==2 && rc_ctrl.rc.s[0]==2)
		{
			//��û��ʶ��Ŀ�꣬��ʼѲ��
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
			
			//��ͷʶ��Ŀ��
			if(Rx_vision.L_tracking == 1)
			{
				//��ͷֹͣѲ��
				nod_flag_L = 0;
				target_pitch_a_L = Rx_vision.L_pitch;
				pitch_control_L(12, -24);
			}
			if(Rx_vision.R_tracking == 1)
			{
				//��ͷֹͣѲ��
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

//**************************************************����ʵ��**************************************************//
static void Pitch_init(void)
{
	initial_pitch_L = 8171;
	initial_pitch_R = 90;
	
	//PID��ʼ��
	pid_init(&pid_pitch_s_L,170,0.3,0,30000,30000); //PID��ʼ�� PI
	pid_init(&pid_pitch_a_L,20,0,30,30000,30000); //PD
	
	pid_init(&pid_pitch_s_R,1,0,0,30000,30000); //PID��ʼ��
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
