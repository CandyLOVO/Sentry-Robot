#include "FrictionTask.h"
#include "testTask.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "user_pid.h"
#include "rc_potocal.h"

#define mocalun_speed 15*15    //Ħ����ת��(����ʵ��������Ŀ��ٵ������٣�
#define K_shoot_rate_correct 1 //��Ƶ��������������ʵ��������Ŀ��ٵ�����Ƶ��
#define C_bopan_block_I 5000   //���̶�ת���������Ժ���ģ�
#define C_bopan_unblock_I 50   //����������ת���������Ժ���ģ�

pidTypeDef motor_m3508_pid[6];
pidTypeDef motor_m2006_pid[8];

//PID��ʼ��
static void Friction_init(void);

//Ħ���ּ��ٸ�ֵ
static void Friction_calc(void);

//Ħ���ּ��ٸ�ֵ 
static void Friction_down(void);

//���̶�ת���
static void Bopan_judge(void);

//Ħ����Pid���ֵ����
extern void can_send_mocalun(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
//����Pid���ֵ�ͷ���
extern void can_send_bopan(int16_t motor1,int16_t motor2);


//����PId����
static void Bopan_calc(void);

//��������
static void Bopan_speed_calc_L(int speed_rate_high, int speed_rate_low,int speed_rate_test);
static void Bopan_speed_calc_R(int speed_rate_high, int speed_rate_low,int speed_rate_test);

//===============================================ȫ�ֱ���================================================//
extern Shooter_t Shooter_L;
extern Shooter_t Shooter_R;
extern RC_ctrl_t rc_ctrl;
extern RC_ctrl_t rc_ctrl;
int16_t bopan_shoot_rate_max = 360;	//�����Ƶ����/min��
int16_t bopan_shoot_rate_min = 240; //�����Ƶ
int16_t bopan_shoot_rate_test = 100;//�޲���ϵͳ��Ƶ
int16_t bopan_reversal_shoot_rate = -100;	//���̷�ת��Ƶ
uint8_t bopan_reversal_flag_L= 0,bopan_reversal_flag_R= 0;	//���̷�ת��־λ��0Ϊ��ת��1Ϊ��ת

void FrictionTask(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	
	Friction_init();	//PID��ʼ��
	
  for(;;)
  {
		//===============================================Ħ����================================================//
		//����Ħ����
		if(1)//Ħ���ֿ�������
		{
		Friction_calc();	//ת��->����
		}
		else//Ħ���ֹر�
		{
			Friction_down();	
		}
		can_send_mocalun(motor_m3508[1].send_I,motor_m3508[2].send_I,motor_m3508[3].send_I,motor_m3508[4].send_I);//Ħ���ֵ�������
		osDelay(1);
		//===============================================����================================================//

		
		
		if(Shooter_L.Fire_Flag==1)//��ǹ�ܷ���
	{  if(bopan_reversal_flag_L==1)
				{
					Bopan_speed_calc_L(bopan_reversal_shoot_rate,bopan_reversal_shoot_rate,bopan_reversal_shoot_rate);//�����Ƶ�������Ƶ���޲���ϵͳ��Ƶ
				}
			else if(bopan_reversal_flag_L==0)
			{
			Bopan_speed_calc_L(bopan_shoot_rate_max,bopan_shoot_rate_min,bopan_shoot_rate_test);		
	   	}
	}
				
	else{
	  Bopan_speed_calc_L(0,0,0);
	}
	
	
	
		if(Shooter_R.Fire_Flag==1)//��ǹ�ܷ���
	{  if(bopan_reversal_flag_R==1)
				{
					Bopan_speed_calc_R(bopan_reversal_shoot_rate,bopan_reversal_shoot_rate,bopan_reversal_shoot_rate);//�����Ƶ�������Ƶ���޲���ϵͳ��Ƶ
				}
			else if(bopan_reversal_flag_R==0)
			{
			Bopan_speed_calc_R(bopan_shoot_rate_max,bopan_shoot_rate_min,bopan_shoot_rate_test);		
	   	}
	}
				
	else{
	  Bopan_speed_calc_R(0,0,0);
	}

		
		//���̵�������
		Bopan_calc();//ת��-->����
		Bopan_judge();//���̶�ת���
		can_send_bopan(motor_m2006[5].send_I,motor_m2006[6].send_I);
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

//===============================================PID��ʼ��================================================//
static void Friction_init()
{
	pid_init(&motor_m3508_pid[1],15,0.8,1);//Ħ����
	pid_init(&motor_m3508_pid[2],15,0.8,1);
	pid_init(&motor_m3508_pid[3],15,0.8,1);
	pid_init(&motor_m3508_pid[4],15,0.8,1);
	
	pid_init(&motor_m2006_pid[5],15,0.8,1);//����(��צ)
	pid_init(&motor_m2006_pid[6],15,0.8,1);//20��0.03��0.5
	
	Shooter_L.shooter_heat=1025;
	Shooter_R.shooter_heat=1025;
}

//==============================================Ħ����ת��->����================================================//
static void Friction_calc()
{
	motor_m3508[1].set_v=mocalun_speed;
	motor_m3508[2].set_v=-mocalun_speed;
	motor_m3508[3].set_v=mocalun_speed;
	motor_m3508[4].set_v=-mocalun_speed;
	
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[1], motor_m3508[1].speed, motor_m3508[1].set_v,5000,2500);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[2], motor_m3508[2].speed, motor_m3508[2].set_v,5000,2500);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[3], motor_m3508[3].speed, motor_m3508[3].set_v,5000,2500);
	motor_m3508[4].send_I = pid_cal_s(&motor_m3508_pid[4], motor_m3508[4].speed, motor_m3508[4].set_v,5000,2500);
}

//===============================================Ħ���ּ��ٵ���================================================//
static void Friction_down()
{
	motor_m3508[1].set_v=0;
	motor_m3508[2].set_v=0;
	motor_m3508[3].set_v=0;
	motor_m3508[4].set_v=0;
	
	
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[1], motor_m3508[1].speed, motor_m3508[1].set_v,5000,2500);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[2], motor_m3508[2].speed, motor_m3508[2].set_v,5000,2500);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[3], motor_m3508[3].speed, motor_m3508[3].set_v,5000,2500);
	motor_m3508[4].send_I = pid_cal_s(&motor_m3508_pid[4], motor_m3508[4].speed, motor_m3508[4].set_v,5000,2500);
}



//===============================================����PID����================================================//
static void Bopan_calc()
{motor_m2006[5].send_I = pid_cal_s(&motor_m2006_pid[5], motor_m2006[5].speed, motor_m2006[5].set_v,5000,2500);
	motor_m2006[6].send_I = pid_cal_s(&motor_m2006_pid[6], motor_m2006[6].speed, motor_m2006[6].set_v,5000,2500);

}
//==========================================�����ٶȼ��㣨�������ƣ�======================================//

static void Bopan_speed_calc_L(int speed_rate_high, int speed_rate_low,int speed_rate_test)//�����������Ƶ�������Ƶ���޲���ϵͳ��Ƶ
{if (Shooter_L.shooter_heat<=300)//����С��300���������Ƶ����
	{motor_m2006[5].set_v=speed_rate_high/8*19*K_shoot_rate_correct;//���ת�٣�rpm��=��Ƶ����/min��/8(һȦ����8������*19��������̼��ٱ� 19��1��
	}
else if(Shooter_L.shooter_heat>300 && Shooter_L.shooter_heat<=360)//300<����<=360��������������Ƶ
{motor_m2006[5].set_v=((speed_rate_low-speed_rate_high)/60*Shooter_L.shooter_heat+6*speed_rate_high-5*speed_rate_low)*K_shoot_rate_correct;
}
else if(Shooter_L.shooter_heat>360 && Shooter_L.shooter_heat<400)//360<����<400,�������Ƶ����
{motor_m2006[5].set_v=speed_rate_low;
}
else if(Shooter_L.shooter_heat==1025)   //�޲���ϵͳʱ��ʼ��ǹ������Ϊ1025�����ٷ���
{
motor_m2006[5].set_v=speed_rate_test/8*19*K_shoot_rate_correct;
}
else                                    //�����������ϵͳ���ϣ�������ͣ
{motor_m2006[5].set_v=0;

}
}

static void Bopan_speed_calc_R(int speed_rate_high, int speed_rate_low,int speed_rate_test)//�����������Ƶ�������Ƶ���޲���ϵͳ��Ƶ
{if (Shooter_R.shooter_heat<=300) //����С��300���������Ƶ����
	{motor_m2006[6].set_v=speed_rate_high/8*19*K_shoot_rate_correct;//���ת�٣�rpm��=��Ƶ����/min��/8(һȦ����8������*19��������̼��ٱ� 19��1��
	}
else if(Shooter_R.shooter_heat>300 && Shooter_R.shooter_heat<=360)//300<����<=360��������������Ƶ
{motor_m2006[6].set_v=((speed_rate_low-speed_rate_high)/60*Shooter_R.shooter_heat+6*speed_rate_high-5*speed_rate_low)*K_shoot_rate_correct;
}
else if(Shooter_R.shooter_heat>360 && Shooter_R.shooter_heat<400)//360<����<400,�������Ƶ����
{motor_m2006[6].set_v=speed_rate_low;
}
else if(Shooter_R.shooter_heat==1025)   //�޲���ϵͳʱ��ʼ��ǹ������Ϊ1025���Բ�����Ƶ����
{
motor_m2006[6].set_v=speed_rate_test/8*19*K_shoot_rate_correct;
}
else                                    //�����������ϵͳ���ϣ�������ͣ
{motor_m2006[6].set_v=0;

}
}
//=====================================================���̶�ת���=======================================//
static void Bopan_judge()
{if(motor_m2006[5].tor_current>C_bopan_block_I)//�޸Ķ�ת����
	{bopan_reversal_flag_L=1;
	}
	else if(0>motor_m2006[5].tor_current && motor_m2006[5].tor_current>-C_bopan_unblock_I)
	{bopan_reversal_flag_L=0;
	
	}		
	
	if(motor_m2006[6].tor_current>C_bopan_block_I)
	{bopan_reversal_flag_R=1;
	}
	else if(0>motor_m2006[6].tor_current && motor_m2006[6].tor_current>-C_bopan_unblock_I)
	{bopan_reversal_flag_R=0;
	}		
	
}

