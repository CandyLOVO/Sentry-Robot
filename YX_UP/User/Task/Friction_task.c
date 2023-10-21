#include "Friction_task.h"

//================================================���������������================================================//
//��һ�棺
//�������дĦ���ֺͲ���
//Ħ���ֵ�ID�ֱ���1��2   ---   ��C��CAN_2
//���̵�ID��5   ---   CAN_1��������ͨ��

//�ڶ��棺
//�����������Ħ���ֺͲ���
//��Ħ����ID�ֱ���3��4   ---   ��C��CAN_2
//�������̣�����ID��6   ---   CAN_1��������ͨ��

//===============================================ȫ�ֱ���================================================//
int16_t bopan_shoot_speed = 90*36;	//���̷��䵯��ת��
int16_t bopan_reversal_speed = -35*36;	//���̷�תת��
uint8_t bopan_reversal_flag = 0;	//���̷�ת��־λ��0Ϊ��Ҫ����ת��1Ϊ��Ҫ��ת

//PID��ʼ��
static void Friction_init();

//Ħ���ּ��ٸ�ֵ
static void Friction_calc();

//Ħ���ּ��ٸ�ֵ
static void Friction_down();

//Ħ���ֳ����ж�
static bool Friction_judge();

//Ħ����Pid���ֵ����
static void Friction_send();

//����Pid���ֵ����ͷ���
static void Bopan_send(int16_t speed);

void Friction_task(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	
	Friction_init();	//PID��ʼ��
	
  for(;;)
  {
		//===============================================Ħ����================================================//
		if(rc_ctrl.rc.s[0] == 2)	//����Ħ����
		{
			Friction_calc();	//ת��->����
		}
		else
		{
			Friction_down();	//Ħ���ּ��ٵ���
		}
		Friction_send();	//Ħ���ֵ�������
		
		//===============================================����================================================//
		if(rc_ctrl.rc.s[1] == 1)	//��������(����ģʽ)
		{	
			if(!bopan_reversal_flag)	//������ת
			{
				Bopan_send(bopan_shoot_speed);
			}
			else if(bopan_reversal_flag)	//���̷�ת
			{
				Bopan_send(bopan_reversal_speed);
			}
			
		}
		else if(rc_ctrl.rc.s[1] == 2  && foe_flag && (Yaw_minipc_fp<5.0f && Yaw_minipc_fp> -5.0f) && (Pitch_minipc_fp<5.0f && Pitch_minipc_fp>-5.0f) && Pitch_minipc_fp!=0 && Yaw_minipc_fp!=0)//��⵽Ŀ��
		{
			if(!bopan_reversal_flag)	//������ת
			{
				Bopan_send(bopan_shoot_speed);
			}
			else if(bopan_reversal_flag)	//���̷�ת
			{
				Bopan_send(bopan_reversal_speed);
			}
		}
		else
		{			
			Bopan_send(0);
		}
		
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

//===============================================PID��ʼ��================================================//
static void Friction_init()
{
	pid_init(&motor_pid_can_2[0],40,0.8,1,16384,16384);//Ħ����
	pid_init(&motor_pid_can_2[1],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[2],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[3],40,0.8,1,16384,16384);
	
	pid_init(&motor_pid[4],20,0.03,0.5,16384,16384);//����(��צ)
	pid_init(&motor_pid[5],20,0.03,0.5,16384,16384);
}

//===============================================Ħ����ת��->����================================================//
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

//===============================================Ħ���ּ��ٵ���================================================//
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

//===============================================Ħ����ת���ж�(δʹ��)================================================//
static bool Friction_judge()
{
	if(	(motor_info_can_2[1].rotor_speed>=8500) && (motor_info_can_2[2].rotor_speed<=-8500) )
	{
		return true;
	}
	return false;
}

//===============================================Ħ���ֵ������ͺ���================================================//
static void Friction_send()
{
		set_motor_voltage_can_2(0, 
                      motor_info_can_2[0].set_voltage, 
                      motor_info_can_2[1].set_voltage, 
                      motor_info_can_2[2].set_voltage, 
                      motor_info_can_2[3].set_voltage);
}

//===============================================���̵������ͺ���================================================//
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
