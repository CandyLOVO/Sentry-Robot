#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "Friction_task.h"
#include "Exchange_task.h"

//��һ�棺
//�������дĦ���ֺͲ���
//Ħ���ֵ�ID�ֱ���1��2   ---   ��C��CAN_2
//���̵�ID��5   ---   CAN_1��������ͨ��

//�ڶ��棺
//�����������Ħ���ֺͲ���
//��Ħ����ID�ֱ���3��4   ---   ��C��CAN_2
//�������̣�����ID��6   ---   CAN_1��������ͨ��

int16_t bopan = -35*36;
uint8_t bopan_fan = 0;

//PID��ʼ��
static void Friction_init();

//Ħ���ּ��ٸ�ֵ
static void Friction_calc();

//Ħ�����ٶ�����
static void Friction_limit();

//Ħ���ּ��ٸ�ֵ
static void Friction_down();

//Ħ���ֳ����ж�
static bool Friction_judeg();

//Ħ����Pid���ֵ����
static void Friction_send();

//����Pid���ֵ����ͷ���
static void Bopan_send(int16_t speed);

void Friction_task(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	
	Friction_init();
  for(;;)
  {
		if(rc_ctrl.rc.s[0] == 2)	//����Ħ����
		{
			Friction_calc();
		}
		else
		{
			Friction_down();
		}
		Friction_limit();
		if(rc_ctrl.rc.s[1] == 1)
		{	
			if(!bopan_fan)
			{
				Bopan_send(90*36);
			}
			else if(bopan_fan)
			{
				Bopan_send(-bopan);
			}
			
		}
		else if(rc_ctrl.rc.s[1] == 2  && foe_flag && (Yaw_minipc_fp<5.0f && Yaw_minipc_fp> -5.0f) && (Pitch_minipc_fp<5.0f && Pitch_minipc_fp>-5.0f) && Pitch_minipc_fp!=0 && Yaw_minipc_fp!=0)//��⵽Ŀ��
		{
			if(!bopan_fan)
			{
				Bopan_send(90*36);
			}
			else if(bopan_fan)
			{
				Bopan_send(-bopan);
			}
		}
		else
		{			
			Bopan_send(0);
		}
		
		Friction_send();

    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

static void Friction_init()
{
	pid_init(&motor_pid_can_2[0],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[1],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[2],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[3],40,0.8,1,16384,16384);
	
	pid_init(&motor_pid[4],20,0.03,0.5,16384,16384);
	pid_init(&motor_pid[5],20,0.03,0.5,16384,16384);
}


static void Friction_calc()
{
	target_speed_can_2[0]=-19*350;
	target_speed_can_2[1]=19*350;
	target_speed_can_2[2]=-19*350;
	target_speed_can_2[3]=19*350;
	
	motor_info_can_2[0].set_voltage = pid_calc(&motor_pid_can_2[0], target_speed_can_2[0], motor_info_can_2[0].rotor_speed);
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[1].rotor_speed);
	motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed);
	motor_info_can_2[3].set_voltage = pid_calc(&motor_pid_can_2[3], target_speed_can_2[3], motor_info_can_2[3].rotor_speed);
}


static void Friction_down()
{
	target_speed_can_2[0]=0;
	target_speed_can_2[1]=0;
	target_speed_can_2[2]=0;
	target_speed_can_2[3]=0;
	motor_info_can_2[0].set_voltage = pid_calc(&motor_pid_can_2[0], target_speed_can_2[0], motor_info_can_2[0].rotor_speed);
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[1].rotor_speed);
	motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed);
	motor_info_can_2[3].set_voltage = pid_calc(&motor_pid_can_2[3], target_speed_can_2[3], motor_info_can_2[3].rotor_speed);
}

static bool Friction_judeg()
{
	if(	(motor_info_can_2[1].rotor_speed>=8500) && (motor_info_can_2[2].rotor_speed<=-8500) )
	{
		return true;
	}
	return false;
}

static void Friction_send()
{
		set_motor_voltage_can_2(0, 
                      motor_info_can_2[0].set_voltage, 
                      motor_info_can_2[1].set_voltage, 
                      motor_info_can_2[2].set_voltage, 
                      motor_info_can_2[3].set_voltage);
}

static void Bopan_send(int16_t speed)
{
		motor_info[4].set_voltage=pid_calc(&motor_pid[4],speed,motor_info[4].rotor_speed);
		motor_info[5].set_voltage=pid_calc(&motor_pid[5],-speed,motor_info[5].rotor_speed);
		set_motor_voltage(1, 
                      motor_info[4].set_voltage, 
                      motor_info[5].set_voltage, 
                      motor_info[6].set_voltage, 
                      0);
}

static void Friction_limit()
{

}