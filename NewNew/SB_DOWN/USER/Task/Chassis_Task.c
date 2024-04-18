#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "pid_user.h"
#include "can_user.h"
#include "rc_potocal.h"
#include "struct_typedef.h"
#include "math.h"

/*********************************************************��������*********************************************************/
#define radius 3.075 // 615mm/2 m
#define cosin 0.707106781187 //����֮���Ŷ�

pidTypeDef pid_3508;
fp32 error_theta; //��̨����ϵ���������ϵ��н�(��ʱΪ0~360��) ���ڽ��պ���Ҫ������theta���д���
int8_t omega = 0; //��ת���Ӽ����еĽ��ٶ� rad/min
int16_t target_speed[4]; //3508Ŀ���ٶ�
int16_t out_speed[4]; //���Ƶ���ֵ

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;
extern up_data Receive;
/**************************************************************************************************************************/

/*********************************************************��������*********************************************************/
void task_init(void);
void Yaw_Diff(void);
void chassis_calculate(int16_t x, int16_t y);
/**************************************************************************************************************************/

/*******************************************************���̿�������*******************************************************/
void Chassis_Task(void const * argument)
{
	task_init();
	
  for(;;)
  {
		//�����������̨���
		Yaw_Diff();
		
		//ң��������ģʽ����->�м䣬��->�м�
		if(rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==3)
		{
			omega = rc_ctrl.rc.ch[4]*0.05; //���ֿ���С����
//			chassis_calculate(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]); //�Ҳ��˿��Ƶ��� ң�����Ҳ���������
			chassis_calculate(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3]); //�󲦸˿��Ƶ���
			for(int i=0;i<4;i++)
			{
				out_speed[i] = pid_cal_s(&pid_3508, motor[i].speed, target_speed[i]);
			}
			can_cmd_send_3508(out_speed[0], out_speed[1], out_speed[2], out_speed[3]);
		}
		
		//�����ϳ�ģʽ����->���£���->����
		else if(rc_ctrl.rc.s[0]==2 && rc_ctrl.rc.s[1]==2)
		{
			omega = 25; //����С����ת��
			chassis_calculate(Receive.nav_vx, Receive.nav_vy); //���뵼��x��yֵ��CAN1����
			for(int i=0;i<4;i++)
			{
				out_speed[i] = pid_cal_s(&pid_3508, motor[i].speed, target_speed[i]);
			}
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
	pid_init(&pid_3508,10,0,0,30000,30000);
}

void Yaw_Diff()
{
	//�����������̨������Ƕ�
	error_theta = -Receive.yaw_value+180; //������̨����̵ļнǣ�ʹ��9025����ֵ�����̴���0~180��0~-180��
	error_theta = error_theta*3.1415926/180; //ת��Ϊ������
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
/*************************************************************************************************************************/
