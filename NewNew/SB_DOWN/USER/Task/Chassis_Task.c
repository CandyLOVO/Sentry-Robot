#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "pid_user.h"
#include "can_user.h"
#include "rc_potocal.h"
#include "struct_typedef.h"
#include "math.h"
#include "Yaw_Task.h"
#include "uart_user.h"
#include "Exchange_Task.h"
#include "math.h"
#include "judge.h"
#include "stdlib.h"

/*********************************************************��������*********************************************************/
#define radius 3.075 // (615mm/2) m
#define cosin 0.707106781187 //����֮���Ŷ�
#define RANDOM_MAX	2500		//��������ֵ
#define RANDOM_MIN  1500  	//�������Сֵ

pidTypeDef pid_3508;
pidTypeDef pid_chassis;
fp32 error_theta; //��̨����ϵ���������ϵ��н�(��ʱΪ0~360��) ���ڽ��պ���Ҫ������theta���д���
float omega = 0; //��ת���Ӽ����еĽ��ٶ� rad/min
float target_speed[4]; //3508Ŀ���ٶ�
int16_t out_speed[4]; //���Ƶ���

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;
extern float yaw_angle;
extern Rx_naving Rx_nav;
extern int8_t flag;
extern double yaw12; //��̨������yawֵ
extern Sentry_t Sentry;
extern RTC_HandleTypeDef hrtc;

float Watch_Power_Max;
float Watch_Power;
float Watch_Buffer;
double Chassis_pidout;
double Chassis_pidout_target;
static double Scaling1=0,Scaling2=0,Scaling3=0,Scaling4=0;
float Klimit=1;
float Plimit=0;
float Chassis_pidout_max;
uint32_t random_value;
RTC_DateTypeDef  date_info;
RTC_TimeTypeDef  time_info;
int count_random = 0;
/**************************************************************************************************************************/

/*********************************************************��������*********************************************************/
void task_init(void);
void Yaw_Diff(void);
void chassis_calculate(int16_t x, int16_t y);
void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);
static void Chassis_Power_Limit(double Chassis_pidout_target_limit);
/**************************************************************************************************************************/

/*******************************************************���̿�������*******************************************************/
void Chassis_Task(void const * argument)
{
	task_init();
	
  for(;;)
  {
		if(flag == 1)
		{
		count_random++;
		if(count_random > 2000)
		{
			//���������
			HAL_RTC_GetTime(&hrtc, &time_info, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &date_info, RTC_FORMAT_BIN);
			srand(time_info.Hours+time_info.Minutes+time_info.Seconds); //�������������
			random_value = rand() % (RANDOM_MAX + 1 - RANDOM_MIN) + RANDOM_MIN;//���������
			count_random = 0;
		}
		
		//�����������̨���
		Yaw_Diff();
		
		//ң��������ģʽ����->�м䣬��->�м�
		if(rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==3)
		{
			omega = rc_ctrl.rc.ch[4]*5; //���ֿ���С����
			chassis_calculate(rc_ctrl.rc.ch[0]*6, rc_ctrl.rc.ch[1]*6); //�Ҳ��˿��Ƶ��� ң�����Ҳ���������
			
			for(int i=0;i<4;i++)
			{
				out_speed[i] = pid_cal_s(&pid_3508, motor[i].speed, target_speed[i]);
			}
		}
		
		//�����ϳ�ģʽ����->���£���->����
		else if(rc_ctrl.rc.s[0]==2 && rc_ctrl.rc.s[1]==2)
		{	
		  if((Sentry.my_outpost_HP == 0) && (Sentry.rfid[1] == 0x02)) //ǰ��վ���Ʋ��һص�Ѳ����
			{
				omega = 2000; //����С����ת��
			}
			else
			{
				if(Sentry.my_outpost_HP != 0) //ǰ��սû�б��� �޵�״̬
				{
					omega = 0;
				}
				else
				{
					if(Rx_nav.nav_x==0 && Rx_nav.nav_y==0) //����ʹ��ͣ��
					{
						omega = 2000; //����С����ת��
					}
					else //������ֵ
					{
						omega = 0;
					}
				}
			}
			
			//�����������±�־λ
			if(Rx_nav.poing == 1)
			{
				omega = pid_cal_a(&pid_chassis, error_theta, 0);
			}
			
			omega = 0; //test
			chassis_calculate(Rx_nav.nav_x, Rx_nav.nav_y); //���뵼��x��yֵ��CAN1����
			for(int i=0;i<4;i++)
			{
				out_speed[i] = pid_cal_s(&pid_3508, motor[i].speed, target_speed[i]);
			}
		}
		
		//���Ե��̸�����̨ģʽ����->���ϣ���->�м�
		else if(rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==1)
		{
			omega = pid_cal_a(&pid_chassis, error_theta, 0);
			chassis_calculate(rc_ctrl.rc.ch[0]*6, rc_ctrl.rc.ch[1]*6); //�Ҳ��˿��Ƶ���
			for(int i=0;i<4;i++)
			{
				out_speed[i] = pid_cal_s(&pid_3508, motor[i].speed, target_speed[i]);
			}
		}
		
		Chassis_Power_Limit(40000);
		can_cmd_send_3508(out_speed[0], out_speed[1], out_speed[2], out_speed[3]);
		}
    osDelay(1);
  }
}
/**************************************************************************************************************************/

/*********************************************************����ʵ��*********************************************************/
void task_init()
{
	//PID������ʼ��
	pid_init(&pid_3508, 30, 0.3, 0, 16384, 16384);
	pid_init(&pid_chassis, 2100, 0.1, 30, 16384, 16384);
	count_random = 0;
}

void Yaw_Diff()
{
	//�����������̨������Ƕ�
	error_theta = yaw_angle; //��̨����̵ļнǣ�ʹ��5010����ֵ��yaw_task�õ���0~180��0~-180��
	error_theta = error_theta*3.1415926/180; //ת��Ϊ������
//	error_theta = 0;
}

void chassis_calculate(int16_t x, int16_t y)
{
	//���̸�����̨������ת����
	int16_t vx = x*cos(error_theta) - y*sin(error_theta); //RPM
	int16_t vy = x*sin(error_theta) + y*cos(error_theta);
	//ȫ�����˶�����
	target_speed[0] = omega*radius + vx*cosin - vy*cosin;
	target_speed[1] = omega*radius - vx*cosin - vy*cosin;
	target_speed[2] = omega*radius - vx*cosin + vy*cosin;
	target_speed[3] = omega*radius + vx*cosin + vy*cosin;
}

//��3508��6020�˶�ģʽ��ϣ��γɵ��̿���
void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed)
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//�����ֵ
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0ת�������ͣ���Ȼ���ܻ�ֱ�ӵ�0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }
    }
}

static void Chassis_Power_Limit(double Chassis_pidout_target_limit)
{	
		//819.2/A�����������Ϊ120W����ô�ܹ�ͨ����������Ϊ5A��ȡһ������ֵ��800.0 * 5 = 4000
		Watch_Power_Max=Klimit;	
		Watch_Power=Sentry.Myself_chassis_power;	
		Watch_Buffer=Sentry.Myself_chassis_power_buffer;//Hero_chassis_power_buffer;//����ֵ������ֵ����������ֵ����ʼֵ��1��0��0

		Chassis_pidout_max=32768;//32768��40��960			15384 * 4��ȡ��4��3508�����������һ������ֵ

		if(Watch_Power>450)
			Motor_Speed_limiting(out_speed,4096);//��������ٶ� ;//5*4*24;������������������ƽ���ı䣬��֪��Ϊɶһ��ʼ�õ�Power>960,���Թ۲������ֵ�������ܲ���ѹե���幦��
		else{
		Chassis_pidout=(
						fabs(out_speed[0]-motor[0].speed)+
						fabs(out_speed[1]-motor[1].speed)+
						fabs(out_speed[2]-motor[2].speed)+
						fabs(out_speed[3]-motor[3].speed));//fabs�������ֵ�������ȡ��4�����ӵĲ�ֵ���

		/*�����ͺ�ռ�Ȼ������������ٶ�*/
		if(Chassis_pidout)
		{
		Scaling1=(out_speed[0]-motor[0].speed)/Chassis_pidout;	
		Scaling2=(out_speed[1]-motor[1].speed)/Chassis_pidout;
		Scaling3=(out_speed[2]-motor[2].speed)/Chassis_pidout;	
		Scaling4=(out_speed[3]-motor[3].speed)/Chassis_pidout;//�������4��scaling���Ϊ1
		}
		else{Scaling1=0.25,Scaling2=0.25,Scaling3=0.25,Scaling4=0.25;}
		
		/*���������ռ�Ȼ�������������ٶ�*/
		Klimit = Chassis_pidout/Chassis_pidout_target_limit;
		
		if(Klimit > 1) Klimit = 1 ;
		else if(Klimit < -1) Klimit = -1;//���ƾ���ֵ���ܳ���1��Ҳ����Chassis_pidoutһ��ҪС��ĳ���ٶ�ֵ�����ܳ���

		/*��������ռ�Ȼ�������Լ��*/
		if(Watch_Buffer<50&&Watch_Buffer>=40)	Plimit=0.8;		//��������һ��������Լ��������Ϊ�˱��ؿ��Ե���Plimit������Ӱ����Ӧ�ٶȣ�
		else if(Watch_Buffer<40&&Watch_Buffer>=35)	Plimit=0.7;
		else if(Watch_Buffer<35&&Watch_Buffer>=30)	Plimit=0.5;
		else if(Watch_Buffer<30&&Watch_Buffer>=20)	Plimit=0.3;
		else if(Watch_Buffer<20&&Watch_Buffer>=10)	Plimit=0.25;
		else if(Watch_Buffer<10&&Watch_Buffer>=0)	Plimit=0.1;
		else {Plimit=1;}
		
		out_speed[0] = Scaling1*(Chassis_pidout_max*Klimit)*Plimit;//���ֵ
		out_speed[1] = Scaling2*(Chassis_pidout_max*Klimit)*Plimit;
		out_speed[2] = Scaling3*(Chassis_pidout_max*Klimit)*Plimit;
		out_speed[3] = Scaling4*(Chassis_pidout_max*Klimit)*Plimit;/*ͬ�����ŵ���*/
	}
}
/*************************************************************************************************************************/
