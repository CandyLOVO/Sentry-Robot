#include "Yaw_task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "fdcan.h"
#include "main.h"
#include "pid_user.h"

pidTypeDef pid_yaw_s;
pidTypeDef pid_yaw_a;
uint8_t can_send_data[8];
int32_t initial_angle_L;
int32_t initial_angle_R;
float yaw_angle_L;
float yaw_angle_R;
int16_t target_yaw_a_L;
int16_t target_yaw_s_L;
int16_t target_yaw_a_R;
int16_t target_yaw_s_R;

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;

//	����ͷyaw�õ�FDCAN1,ID��1��2 ������ID�д����ԣ�ֻ��Ҫ����motor[]�е���š�
void Yaw_Task(void const * argument)
{
	Yaw_init();
	osDelay(3000);
  for(;;)
  {
		//����yaw�ĳ�ʼλ��ӳ��yaw�ĽǶ�
		yaw_angle_L = motor_value(initial_angle_L, motor[0].angle);
		yaw_angle_R = motor_value(initial_angle_R, motor[1].angle);
		
		//ң������Сyawģʽ����->�У���->��
		if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==1)
		{
			yaw_control_L(-170,-10);
			yaw_control_R(0,0);
		}
		osDelay(1);
  }
}

//================================================YAW��PID������Ŀ��IMU��ʼ��================================================//
static void Yaw_init(void)
{
	initial_angle_L = 8192;
	initial_angle_R = 8192;
	target_yaw_a_L = 0;
	target_yaw_s_L = 0;
	target_yaw_a_R = 0;
	target_yaw_s_R = 0;
	pid_init(&pid_yaw_s,10,0,0,30000,30000); //PID��ʼ��
	pid_init(&pid_yaw_a,1,0,0,30000,30000);
}

void yaw_control_L(int16_t max_angle, int16_t min_angle)
{
	int16_t error;
	
	//ң��������
	target_yaw_a_L += rc_ctrl.rc.ch[2] * 30/660;
	
	//Խ�紦��
	if(target_yaw_a_L>180)
	{
		target_yaw_a_L -= 360;
	}
	else if(target_yaw_a_L<-180)
	{
		target_yaw_a_L += 360;
	}
	
	//�����λ
	if((target_yaw_a_L>min_angle) && (target_yaw_a_L<max_angle))
	{
		if((yaw_angle_L>min_angle) && (yaw_angle_L<max_angle))
		{
			if(motor[0].speed>0)
			{
				target_yaw_a_L = min_angle;
			}
			else if(motor[0].speed<0)
			{
				target_yaw_a_L = max_angle;
			}
		}
	}
	
}

void yaw_control_R(int16_t max_angle, int16_t min_angle)
{
	//ң��������
	target_yaw_a_R += rc_ctrl.rc.ch[0] * 30/660;
	
	//Խ�紦��
	if(target_yaw_a_R>180)
	{
		target_yaw_a_R -= 360;
	}
	else if(target_yaw_a_R<-180)
	{
		target_yaw_a_R += 360;
	}
	
	//�����λ
	if(target_yaw_a_R>max_angle)
	{
		target_yaw_a_R = max_angle;
	}
	else if(target_yaw_a_R<min_angle)
	{
		target_yaw_a_R = min_angle;
	}
}

//��6020����ĽǶ��Գ�ʼ�Ƕ�Ϊ0��ӳ�䵽0~+-180�� (k:�趨�ĳ�ʼ�Ƕȡ�0�� �� n:��Ҫӳ��ĽǶ�)
//�ý���Ϊ��װ6020��ӳ�䣬��װ6020��Ҫ���������ӡ�-��
float motor_value(int16_t k, int16_t n)
{
	float ans;
	if(k>=0 && k<4096){
		if(n>=0 && n<(k+4096)){
			n = k - n;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
		else if(n>=(k+4096) && n<8191){
			n = 8192 - n + k;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
	}
	
	else if(k>=4096 && k<8192){
		if(n>=0 && n<(k-4096)){
			n = -8192 + k - n;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
		else if(n>=(k-4096) && n<8191){
			n = k - n;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
	}
}
