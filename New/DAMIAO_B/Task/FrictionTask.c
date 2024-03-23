#include "FrictionTask.h"
#include "testTask.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "user_pid.h"
#include "rc_potocal.h"
#include "user_tim.h"


#define mocalun_speed   19*380    //Ħ����ת��(����ʵ��������Ŀ��ٵ������٣�
#define C_bopan_reversal_time 0.8f   //���̷�תʱ��(s)
#define K_shoot_rate_correct 1 //��Ƶ��������������ʵ��������Ŀ��ٵ�����Ƶ��
#define C_bopan_block_I 9000   //���̶�ת���������Ժ���ģ�
#define K_rc_to_bopanSpeed 6 //ң��ͨ��ֵ�л��������ٶȣ����Ŀɿ��ٵ���1-1ģʽ��ң���벦��ӳ���ϵ��


pidTypeDef motor_m3508_pid[4];
pidTypeDef motor_m2006_pid[2];



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

//ң�ؿ���
static void re_control(void);
static void re_shoot_rate_calc(void);

//===============================================ȫ�ֱ���================================================//
extern Shooter_t Shooter_L;
extern Shooter_t Shooter_R;
extern RC_ctrl_t rc_ctrl;
extern RC_ctrl_t rc_ctrl;
int16_t bopan_shoot_rate_max = 360;	//�����Ƶ����/min��
int16_t bopan_shoot_rate_min = 240; //�����Ƶ
int16_t bopan_shoot_rate_test = 150;//�޲���ϵͳ��Ƶ
int16_t bopan_reversal_shoot_rate = -100;	//���̷�ת��Ƶ
uint8_t bopan_reversal_flag_L= 0,bopan_reversal_flag_R= 0;	//���̷�ת��־λ��0Ϊ��ת��1Ϊ��ת
int16_t remote_mode;         //ң��ģʽ��1-1ΪFire Control ģʽ��Ħ������ת��ң������ҡ�˿��Ʋ�����ת��2-2Ϊ�ϳ�ģʽ��Ħ������ת�������Ӿ�ʶ��λ����
extern float Sys_time;


void FrictionTask(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	
	Friction_init();	//PID��ʼ��
	
  for(;;)
  {
		re_control();
		re_shoot_rate_calc();
		//===============================================Ħ����================================================//
		//����Ħ����
		if(remote_mode==22 || remote_mode==11)//Ħ���ֿ�������
		{
		Friction_calc();	//ת��->����
		}
		else//Ħ���ֹر�
		{
			Friction_down();	
		}
		can_send_mocalun(motor_m3508[0].send_I,motor_m3508[1].send_I,motor_m3508[2].send_I,motor_m3508[3].send_I);//Ħ���ֵ�������
		//===============================================����================================================//
      //===========================================�Զ�ģʽ begin================================//
if(remote_mode==22)
{
		
		if(Shooter_L.Fire_Flag==1 )//��ǹ�ܷ���
	{  
		if(bopan_reversal_flag_L==1)
			{
			Bopan_speed_calc_L(bopan_reversal_shoot_rate,bopan_reversal_shoot_rate,bopan_reversal_shoot_rate);
			}
			else if(bopan_reversal_flag_L==0)
			{
			Bopan_speed_calc_L(bopan_shoot_rate_max,bopan_shoot_rate_min,bopan_shoot_rate_test);	//�����Ƶ�������Ƶ���޲���ϵͳ��Ƶ	
	   	}
	}
				
	  else
	{
	  Bopan_speed_calc_L(0,0,0);
	}
	
	
	
		if(Shooter_R.Fire_Flag==1)//��ǹ�ܷ���
	{  if(bopan_reversal_flag_R==1)
		  {
			Bopan_speed_calc_R(bopan_reversal_shoot_rate,bopan_reversal_shoot_rate,bopan_reversal_shoot_rate);//�����Ƶ�������Ƶ���޲���ϵͳ��Ƶ
			}
		 else if(bopan_reversal_flag_R==0)
			{
			Bopan_speed_calc_R(bopan_shoot_rate_max,bopan_shoot_rate_min,bopan_shoot_rate_test);		//�����Ƶ�������Ƶ���޲���ϵͳ��Ƶ
	   	}
	}
				
	  else
  {
	  Bopan_speed_calc_R(0,0,0);
	}
	
}

//========================ң��ģʽ==============================//  
		
else if (remote_mode==11)
{
		re_shoot_rate_calc();  //ң�ؿ��Ʋ��̣�����ҡ������
}

else
{
Bopan_speed_calc_L(0,0,0);
Bopan_speed_calc_R(0,0,0);
}
		            
	  Bopan_judge();//���̶�ת���
    Bopan_calc();//ת��-->����
		can_send_bopan(motor_m2006[0].send_I,motor_m2006[1].send_I);//���̵�������
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

//===================================================ң�ؿ���===============================================//
static void re_control()
{
		if((rc_ctrl.rc.s[1]==2 && rc_ctrl.rc.s[0]==2)||(rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==2))
		{
		remote_mode=22;
		}
    else if (rc_ctrl.rc.s[1]==1 &&rc_ctrl.rc.s[0]==1)
		{
		remote_mode=11;
		}
		else
		{
		remote_mode=0;
		}
}

static void re_shoot_rate_calc()
{
	motor_m2006[0].set_v=rc_ctrl.rc.ch[3]*K_rc_to_bopanSpeed;
	motor_m2006[1].set_v=rc_ctrl.rc.ch[1]*K_rc_to_bopanSpeed;

}

//===============================================PID��ʼ��================================================//
static void Friction_init()
{
	pid_init(&motor_m3508_pid[0],40,0.8,1);//Ħ����
	pid_init(&motor_m3508_pid[1],40,0.8,1);
	pid_init(&motor_m3508_pid[2],40,0.8,1);
	pid_init(&motor_m3508_pid[3],40,0.8,1);
	
	pid_init(&motor_m2006_pid[0],20,0.03,0.5);//����()
	pid_init(&motor_m2006_pid[1],20,0.03,0.5);//20��0.03��0.5
	
	Shooter_L.shooter_heat=1025;
	Shooter_R.shooter_heat=1025;
	motor_m3508[0].set_v = 0;
	motor_m3508[1].set_v =0;
	motor_m3508[2].set_v = 0;
	motor_m3508[3].set_v =0;
}

//==============================================Ħ����ת��->����================================================//
static void Friction_calc()
{
	if(motor_m3508[0].set_v>-mocalun_speed)
	{
	motor_m3508[0].set_v -= 10;
	
	}
	else
	{
	motor_m3508[0].set_v = -mocalun_speed;
	}
	
		if(motor_m3508[1].set_v<mocalun_speed)
	{
	motor_m3508[1].set_v += 10;
	
	}
	else
	{
	motor_m3508[1].set_v = mocalun_speed;
	}
	
		if(motor_m3508[2].set_v>-mocalun_speed)
	{
	motor_m3508[2].set_v -= 10;
	
	}
	else
	{
	motor_m3508[2].set_v = -mocalun_speed;
	}
	
		if(motor_m3508[3].set_v<mocalun_speed)
	{
	motor_m3508[3].set_v += 10;
	
	}
	else
	{
	motor_m3508[3].set_v = mocalun_speed;
	}
	

	
	motor_m3508[0].send_I = pid_cal_s(&motor_m3508_pid[0], motor_m3508[0].speed, motor_m3508[0].set_v,16384,16384);
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[1], motor_m3508[1].speed, motor_m3508[1].set_v,16384,16384);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[2], motor_m3508[2].speed, motor_m3508[2].set_v,16384,16384);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[3], motor_m3508[3].speed, motor_m3508[3].set_v,16384,16384);

}

//===============================================Ħ���ּ��ٵ���================================================//
static void Friction_down()
{
	motor_m3508[0].set_v = 0;
	motor_m3508[1].set_v = 0;
	motor_m3508[2].set_v = 0;
	motor_m3508[3].set_v = 0;
	motor_m3508[0].send_I = 0;
	motor_m3508[1].send_I = 0;
	motor_m3508[2].send_I = 0;
	motor_m3508[3].send_I = 0;
}   



//===============================================����PID����================================================//
static void Bopan_calc()
{	
	motor_m2006[0].send_I = pid_cal_s(&motor_m2006_pid[0], motor_m2006[0].speed, -motor_m2006[0].set_v,16384,16384);
	motor_m2006[1].send_I = pid_cal_s(&motor_m2006_pid[1], motor_m2006[1].speed, -motor_m2006[1].set_v,16384,16384);
}
//==========================================�����ٶȼ��㣨�������ƣ�======================================//

static void Bopan_speed_calc_L(int speed_rate_high, int speed_rate_low,int speed_rate_test)//�����������Ƶ�������Ƶ���޲���ϵͳ��Ƶ
{
	if (Shooter_L.shooter_heat<=300)//����С��300���������Ƶ����
	{
		motor_m2006[0].set_v=speed_rate_high/8*36*K_shoot_rate_correct;//���ת�٣�rpm��=��Ƶ����/min��/8(һȦ����8������*36��������̼��ٱ� 36��1��
	}
	else if(Shooter_L.shooter_heat>300 && Shooter_L.shooter_heat<=360)//300<����<=360��������������Ƶ
	{
		motor_m2006[0].set_v=((speed_rate_low-speed_rate_high)/60*Shooter_L.shooter_heat+6*speed_rate_high-5*speed_rate_low)/8*36*K_shoot_rate_correct;
}
else if(Shooter_L.shooter_heat>360 && Shooter_L.shooter_heat<400)//360<����<400,�������Ƶ����
{
	motor_m2006[0].set_v=speed_rate_low/8*36*K_shoot_rate_correct;
}
else if(Shooter_L.shooter_heat==1025)   //�޲���ϵͳʱ��ʼ��ǹ������Ϊ1025�����ٷ���
{
	motor_m2006[0].set_v=speed_rate_test/8*36*K_shoot_rate_correct;
}
else                                    //�����������ϵͳ���ϣ�������ͣ
{
	motor_m2006[0].set_v=0;
}
}

static void Bopan_speed_calc_R(int speed_rate_high, int speed_rate_low,int speed_rate_test)//�����������Ƶ�������Ƶ���޲���ϵͳ��Ƶ
{
	if (Shooter_R.shooter_heat<=300) //����С��300���������Ƶ����
	{
		motor_m2006[1].set_v=speed_rate_high/8*36*K_shoot_rate_correct;//���ת�٣�rpm��=��Ƶ����/min��/8(һȦ����8������*36��������̼��ٱ� 36��1��
	}
	else if(Shooter_R.shooter_heat>300 && Shooter_R.shooter_heat<=360)//300<����<=360��������������Ƶ
	{
		motor_m2006[1].set_v=((speed_rate_low-speed_rate_high)/60*Shooter_R.shooter_heat+6*speed_rate_high-5*speed_rate_low)/8*36*K_shoot_rate_correct;
	}
	else if(Shooter_R.shooter_heat>360 && Shooter_R.shooter_heat<400)//360<����<400,�������Ƶ����
	{
		motor_m2006[1].set_v=speed_rate_low/8*36;
	}
	else if(Shooter_R.shooter_heat==1025)   //�޲���ϵͳʱ��ʼ��ǹ������Ϊ1025���Բ�����Ƶ����
	{
		motor_m2006[1].set_v=speed_rate_test/8*36*K_shoot_rate_correct;
	}
	else                                    //�����������ϵͳ���ϣ�������ͣ
	{
		motor_m2006[1].set_v=0;
	}
}
//=====================================================���̶�ת���=======================================//
static void Bopan_judge()
{if(motor_m2006[0].tor_current<-C_bopan_block_I)
	{
		bopan_reversal_flag_L=1;
		Shooter_L.tim_reversal_begin=Sys_time;
	}
else if(Sys_time-Shooter_L.tim_reversal_begin> C_bopan_reversal_time)
{
		bopan_reversal_flag_L=0;
		Shooter_L.tim_reversal_begin=0;
}
	
	if(motor_m2006[1].tor_current<-C_bopan_block_I)
	{
		bopan_reversal_flag_R=1;
		Shooter_R.tim_reversal_begin=Sys_time;
	}
	else if(Sys_time-Shooter_R.tim_reversal_begin> C_bopan_reversal_time)
{
		bopan_reversal_flag_R=0;
		Shooter_R.tim_reversal_begin=0;
}




	
}

