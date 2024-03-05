#include "FrictionTask_v2.h"
#include "testTask.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "user_pid.h"
#include "rc_potocal.h"

#define K_shoot_speed_correct 1 //������������������ʵ��������Ŀ��ٵ������٣� 
#define K_shoot_rate_correct 1 //��Ƶ��������������ʵ��������Ŀ��ٵ�����Ƶ��
#define C_bopan_block_I 5000   //���̶�ת���������Ժ���ģ�
#define C_speed_protect 0.2    //�������ࣨm/s��
#define C_heat_protect 20      //��������
#define C_bopan_unblock_I 50   //����������ת���������Ժ���ģ�

extern pidTypeDef motor_m3508_pid[4]; 
extern pidTypeDef motor_m2006_pid[2];

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
static void Bopan_speed_calc_L(void);
static void Bopan_speed_calc_R(void);

//===============================================ȫ�ֱ���================================================//
extern Shooter_t Shooter_L;
extern Shooter_t Shooter_R;
extern RC_ctrl_t rc_ctrl;
int16_t bopan_shoot_speed ;	//���̵������ת��
int16_t bopan_reversal_speed = -19*15;	//���̷�תת��
extern uint8_t bopan_reversal_flag_L;
extern uint8_t bopan_reversal_flag_R;	//���̷�ת��־λ��0Ϊ��ת��1Ϊ��ת
int I_test;
void FrictionTask_v2(void const * argument)
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
		can_send_mocalun(motor_m3508[0].send_I,motor_m3508[1].send_I,motor_m3508[2].send_I,motor_m3508[3].send_I);//Ħ���ֵ�������
		osDelay(1);
		
		//===============================================����================================================//		
		if(Shooter_L.Fire_Flag==1)//��ǹ�ܷ���
	{  if(bopan_reversal_flag_L==1)
				{
					motor_m2006[0].set_v=bopan_reversal_speed;//2006Ŀ���ٶ�
				}
			else if(bopan_reversal_flag_L==0)
			{
				
			Bopan_speed_calc_L();		
	   	}
	}
				
	else{
	  motor_m2006[0].set_v=0;
	}
	
	
	
	if(Shooter_R.Fire_Flag==1)//��ǹ�ܷ���
	{  if(bopan_reversal_flag_R==1)
				{
					motor_m2006[1].set_v=bopan_reversal_speed;//2006Ŀ���ٶ�
				}
			else if(bopan_reversal_flag_R==0)
			{
				
			Bopan_speed_calc_R();		
	   	}
	}
				
	else{
	  motor_m2006[1].set_v=0;
	}

		
		//���̵�������
		Bopan_calc();//ת��-->����
		Bopan_judge();//���̶�ת���
		can_send_bopan(motor_m2006[0].send_I,motor_m2006[1].send_I);
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

//===============================================PID��ʼ��================================================//
static void Friction_init()
{
	pid_init(&motor_m3508_pid[0],15,0.8,1);//Ħ����
	pid_init(&motor_m3508_pid[1],15,0.8,1);
	pid_init(&motor_m3508_pid[2],15,0.8,1);
	pid_init(&motor_m3508_pid[3],15,0.8,1);
	
	pid_init(&motor_m2006_pid[0],15,0.8,1);//����(��צ)
	pid_init(&motor_m2006_pid[1],15,0.8,1);//20��0.03��0.5
	

}

//==============================================Ħ����ת��->����================================================//
static void Friction_calc()
{
	int target_shoot_speed_L,target_shoot_speed_R;
	target_shoot_speed_L=(Shooter_L.speed_limit-C_speed_protect)*K_shoot_speed_correct;  //Ŀ������=����������-�������ࣩ*��������ϵ��
	target_shoot_speed_R=(Shooter_R.speed_limit-C_speed_protect)*K_shoot_speed_correct;
	
	motor_m3508[0].set_v=target_shoot_speed_L;
	motor_m3508[1].set_v=-target_shoot_speed_L;
	motor_m3508[2].set_v=target_shoot_speed_R;
	motor_m3508[3].set_v=-target_shoot_speed_R;
	
/*	motor_m3508[0].send_I = pid_cal_s(&motor_m3508_pid[0], Shooter_L.shoot_speed, motor_m3508[0].set_v,5000,2500);
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[1], Shooter_L.shoot_speed, motor_m3508[1].set_v,5000,2500);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[2], Shooter_R.shoot_speed, motor_m3508[2].set_v,5000,2500);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[3], Shooter_R.shoot_speed, motor_m3508[3].set_v,5000,2500);*/
	motor_m3508[0].send_I = I_test;
	motor_m3508[1].send_I = I_test;
	motor_m3508[2].send_I = I_test;
	motor_m3508[3].send_I = I_test;
}

//===============================================Ħ���ּ��ٵ���================================================//
static void Friction_down()
{
	motor_m3508[0].set_v=0;
	motor_m3508[1].set_v=0;
	motor_m3508[2].set_v=0;
	motor_m3508[3].set_v=0;
	
	
	motor_m3508[0].send_I = pid_cal_s(&motor_m3508_pid[1], motor_m3508[1].speed, motor_m3508[1].set_v,5000,2500);
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[2], motor_m3508[2].speed, motor_m3508[2].set_v,5000,2500);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[3], motor_m3508[3].speed, motor_m3508[3].set_v,5000,2500);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[4], motor_m3508[4].speed, motor_m3508[4].set_v,5000,2500);
}



//===============================================����PID����================================================//
static void Bopan_calc()
{ 
	motor_m2006[0].send_I = pid_cal_s(&motor_m2006_pid[0], Shooter_L.shoot_rate, motor_m2006[0].set_v,5000,2500);
	motor_m2006[1].send_I = pid_cal_s(&motor_m2006_pid[1], Shooter_R.shoot_rate, motor_m2006[1].set_v,5000,2500);
}
//==========================================�����ٶȼ��㣨�������ƣ�======================================//

static void Bopan_speed_calc_L()
{int target_shoot_rate_L;  //Ŀ����Ƶ
	int heat_space;          //���������ռ�
	heat_space=Shooter_L.heat_limit-Shooter_L.shooter_heat; //���������ռ�=��������-ʵʱ����
	if(heat_space>=100)
	{
	target_shoot_rate_L=Shooter_L.cooling_rate/10*2;
	}
	else if(100>heat_space && heat_space>=50)
	{
	target_shoot_rate_L=Shooter_L.cooling_rate/10;
	
	}
	else if(50>heat_space && heat_space>=C_heat_protect)
 {
	target_shoot_rate_L=Shooter_L.cooling_rate/10*0.8;
	}
 else
 {
 target_shoot_rate_L=0;
 }
	
motor_m2006[0].set_v=target_shoot_rate_L*K_shoot_rate_correct;
	
}


static void Bopan_speed_calc_R()
{int target_shoot_rate_R;  //Ŀ����Ƶ
	int heat_space;          //���������ռ�
	heat_space=Shooter_R.heat_limit-Shooter_R.shooter_heat; //���������ռ�=��������-ʵʱ����
	if(heat_space>100)
	{
	target_shoot_rate_R=Shooter_R.cooling_rate/10*2;
	}
	else if(100>heat_space && heat_space>50)
	{
	target_shoot_rate_R=Shooter_R.cooling_rate/10;
	
	}
	else if(50>heat_space && heat_space>=5)
 {
	target_shoot_rate_R=Shooter_R.cooling_rate/10*0.8;
	
	}
 else
 {
 target_shoot_rate_R=0;
 }
	
motor_m2006[1].set_v=target_shoot_rate_R*K_shoot_rate_correct;//Ŀ����Ƶ*��Ƶ����ϵ��
	
}

//==========================================���̶�ת���==========================================//
static void Bopan_judge()
{if(motor_m2006[0].tor_current>C_bopan_block_I)//�޸Ķ�ת����
	{bopan_reversal_flag_L=1;
	}
	else if(motor_m2006[0].tor_current<0 && motor_m2006[0].tor_current>-C_bopan_unblock_I)
	{bopan_reversal_flag_L=0;
	
	}		
	
	if(motor_m2006[1].tor_current>C_bopan_block_I)
	{bopan_reversal_flag_R=1;
	}
	else if(motor_m2006[1].tor_current<0 && motor_m2006[1].tor_current>-C_bopan_unblock_I)
	{bopan_reversal_flag_R=0;
	}		
	
}

