#include "Yaw_task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "fdcan.h"
#include "main.h"
#include "pid_user.h"
#include "Exchange_task.h"
#include "imu_temp_ctrl.h"

//**************************************************��������**************************************************//
pidTypeDef pid_yaw_s_L;
pidTypeDef pid_yaw_a_L;
pidTypeDef pid_yaw_s_L_nan;
pidTypeDef pid_yaw_a_L_nan;
pidTypeDef pid_yaw_s_R;
pidTypeDef pid_yaw_a_R;
uint8_t can_send_data[8];
int32_t initial_angle_L;
int32_t initial_angle_R;
float yaw_angle_L;
float yaw_angle_R;
float target_yaw_a_L;
float target_yaw_s_L;
float target_yaw_a_R;
float target_yaw_s_R;
int16_t yaw_output_L;
int16_t yaw_output_R;
uint8_t warning_flag_L = 0; //�жϵ�ǰ�Ƕ��Ƿ���Σ�սǶȵı�־λ
uint8_t warning_flag_R = 0;
int16_t up_limit_L;
int16_t low_limit_L;
int16_t up_limit_R;
int16_t low_limit_R;
uint8_t gimbal_control_6020[8];
uint8_t rotate_flag_L = 0; //�ж��Ƿ���Ҫ��ת
uint8_t rotate_flag_R = 0;
uint8_t heart_direction[4];
float last_target_yaw_a_L;
float last_target_yaw_a_R;
uint16_t time_delay;
uint8_t flag_suo;

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;
extern receive_vision Rx_vision;
extern double yaw12;
extern int8_t flag;
extern uint8_t flag_heart; //�ܻ���װ�װ�ID
extern TIM_HandleTypeDef htim5;
//************************************************************************************************************//

//**************************************************����ʵ��**************************************************//

//��ͷ����IDΪ2����ͷ���ң�IDΪ1
//����ͷyaw�õ�FDCAN1,ID��1��2 ������ID�д����ԣ���Ҫ����motor[]�е���ţ�����CAN���͵���������ݵ�˳��
void Yaw_Task(void * argument)
{
	Yaw_init();
//	osDelay(3000);
  for(;;)
  {
		//�ж��������Ƿ��²�����
		if(flag == 1)
		{
		//����yaw�ĳ�ʼλ��ӳ��yaw�ĽǶ�
		yaw_angle_L = -motor_value(initial_angle_L, motor[1].angle); //��Ҫ����ֵ������
		yaw_angle_R = -motor_value(initial_angle_R, motor[0].angle);
		
		//ң������Сyawģʽ����->�У���->��
		if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==1)
		{
			target_yaw_a_L += rc_ctrl.rc.ch[2] * 0.2/660;
			target_yaw_a_R += rc_ctrl.rc.ch[0] * 0.2/660;
			yaw_control_L(-20, -175); //СyawĿ��ֵ�����λ
			yaw_control_R(175, 20);
		}
		
		if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==3)
		{
			target_yaw_a_L = 0;
			target_yaw_a_R = 0;
			yaw_control_L(-20, -175); //СyawĿ��ֵ�����λ
			yaw_control_R(175, 20);
		}
		
		//����ģʽ����->�£���->��
		if(rc_ctrl.rc.s[1]==2 && rc_ctrl.rc.s[0]==2)
		{
			//�ĸ�����ͷ��û��ʶ��
			if(Rx_vision.L_tracking==0 && Rx_vision.R_tracking==0 && Rx_vision.M_tracking==0)
			{
				if((flag_suo == 1)&&(time_delay <= 1000)) //��һ��״̬Ϊ��ס����1000ms�ڣ�
				{
					rotate_flag_L = 0; //��ͷ��ת
					rotate_flag_R = 0; //��ͷ��ת
					target_yaw_a_L = last_target_yaw_a_L; //Ŀ��Ƕ�Ϊ��סʱ�ĽǶ�
					target_yaw_a_R = last_target_yaw_a_R;
					yaw_control_L(-20, -175); //СyawĿ��ֵ�����λ
					yaw_control_R(175, 20);
				}
				else
				{
					if(flag_heart == 1)
					{
						target_yaw_a_L = 0;
						target_yaw_a_R = 0;
						yaw_control_L(-20, -175); //СyawĿ��ֵ�����λ
						yaw_control_R(175, 20);
					}
					else
					{
						//ʹСyaw��ʼ��ת
						if(rotate_flag_L == 0)
						{
							rotate_flag_L = 1;
						}
						if(rotate_flag_R == 0)
						{
							rotate_flag_R = 1;
						}
						//Сyaw����תѲ��
						yaw_finding_L(-20, -175);
						yaw_finding_R(175, 20);
					}
				}
			}
			else
			{
				//��ͷʶ��
				if(Rx_vision.L_tracking == 1 && Rx_vision.R_tracking == 0)
				{
					rotate_flag_L = 0; //��ͷ��ת
					target_yaw_a_L = Rx_vision.L_yaw - yaw12; //��ͷĿ��ֵ��������ϵת��
					last_target_yaw_a_L = target_yaw_a_L;
					yaw_control_L(-20, -175); //��ͷĿ��ֵ�����λ
					
					rotate_flag_R = 0; //��ͷ��ת
					target_yaw_a_R = -target_yaw_a_L;
					yaw_control_R(175, 20); //��ͷĿ��ֵ�����λ
				}
				//��ͷʶ��
				else if(Rx_vision.R_tracking == 1 && Rx_vision.L_tracking == 0)
				{
					rotate_flag_R = 0; //��ͷ��ת
					target_yaw_a_R = Rx_vision.R_yaw - yaw12; //��ͷĿ��ֵ��������ϵת��
					last_target_yaw_a_R = target_yaw_a_R;
					yaw_control_R(175, 20); //��ͷĿ��ֵ�����λ
					
					rotate_flag_L = 0; //��ͷ��ת
					target_yaw_a_L = 0;
					yaw_control_L(-20, -175); //��ͷĿ��ֵ�����λ
				}
				else if(Rx_vision.L_tracking == 1 && Rx_vision.R_tracking == 1)
				{
					rotate_flag_L = 0; //��ͷ��ת
					target_yaw_a_L = Rx_vision.L_yaw - yaw12; //��ͷĿ��ֵ��������ϵת��
					last_target_yaw_a_L = target_yaw_a_L;
					yaw_control_L(-20, -175); //��ͷĿ��ֵ�����λ
					
					rotate_flag_R = 0; //��ͷ��ת
					target_yaw_a_R = Rx_vision.R_yaw - yaw12; //��ͷĿ��ֵ��������ϵת��
					last_target_yaw_a_R = target_yaw_a_R;
					yaw_control_R(175, 20); //��ͷĿ��ֵ�����λ
				}
				else if(Rx_vision.L_tracking == 0 && Rx_vision.R_tracking == 0)
				{
					//��yaw�ϵ�����ͷʶ��
					if(Rx_vision.M_tracking == 1)
					{
						rotate_flag_L = 0; //��ͷ��ת
						target_yaw_a_L = Rx_vision.L_yaw - yaw12;
						last_target_yaw_a_L = target_yaw_a_L;
						yaw_control_L(-20, -175);
						
						rotate_flag_R = 0; //��ͷ��ת
						target_yaw_a_R = 0;
						yaw_control_R(175, 20); //��ͷĿ��ֵ�����λ
					}
				}
			}
		}
		
		//PID����
		target_yaw_s_L = pid_cal_yaw_a_for_nan(&pid_yaw_a_L_nan, yaw_angle_L, target_yaw_a_L, warning_flag_L, up_limit_L, low_limit_L);
		yaw_output_L = pid_cal_s(&pid_yaw_s_L_nan, motor[1].speed, target_yaw_s_L);
		target_yaw_s_R = pid_cal_yaw_a(&pid_yaw_a_R, yaw_angle_R, target_yaw_a_R, warning_flag_R, up_limit_R, low_limit_R);
		yaw_output_R = pid_cal_s(&pid_yaw_s_R, motor[0].speed, target_yaw_s_R);
		
		//Сyaw6020������ķ��� CAN1
//		gimbal_control_6020[0] = (yaw_output_L>>8)&0xff;
//		gimbal_control_6020[1] = yaw_output_L&0xff;
//		gimbal_control_6020[2] = (yaw_output_R>>8)&0xff;
//		gimbal_control_6020[3] = yaw_output_R&0xff;
		
		gimbal_control_6020[2] = (yaw_output_L>>8)&0xff;
		gimbal_control_6020[3] = yaw_output_L&0xff;
		gimbal_control_6020[0] = (yaw_output_R>>8)&0xff;
		gimbal_control_6020[1] = yaw_output_R&0xff;
		canx_send_data(&hfdcan1, 0x1FF, gimbal_control_6020, 8);
		osDelay(1);
		}
	}
}
//************************************************************************************************************//

//**************************************************����ʵ��**************************************************//
static void Yaw_init(void)
{
	//Сyaw��ʼ��
	initial_angle_L = 3714;
	initial_angle_R = 7835;
	target_yaw_a_L = 0;
	target_yaw_s_L = 0;
	target_yaw_a_R = 0;
	target_yaw_s_R = 0;
	
	//PID��ʼ��
//	pid_init(&pid_yaw_s_L,350,0.08,0,30000,30000); //PID��ʼ�� PI
//	pid_init(&pid_yaw_a_L,8,0,0,30000,30000); //PD
	pid_init(&pid_yaw_s_L,1,0,0,30000,30000); //PID��ʼ�� PI
	pid_init(&pid_yaw_a_L,1,0,0,30000,30000); //PD
	
	pid_init(&pid_yaw_s_L_nan,400,0.2,0,30000,30000); //PID��ʼ�� PI
	pid_init(&pid_yaw_a_L_nan,12,0,0,30000,30000); //PD
	
	pid_init(&pid_yaw_s_R,500,0.2,0,30000,30000); //PID��ʼ��
	pid_init(&pid_yaw_a_R,10,0,0,30000,30000);
	
	heart_direction[0] = 0;
	heart_direction[1] = 90;
	heart_direction[2] = 180;
	heart_direction[3] = -90;
}

void yaw_control_L(float max_angle, float min_angle)
{
	//����Сyaw��max_angle��min_angle�ڵ�����Ϊ���� ���������λ�����max_angle��min_angle��Ϊ�����������
	//�ú�����Ҫ���pid_user.c��pid_cal_yaw_a����ʹ��
	
	warning_flag_L = 0;
	
	//Խ�紦��
	if(target_yaw_a_L>180)
	{
		target_yaw_a_L -= 360;
	} 
	if(target_yaw_a_L<-180)
	{
		target_yaw_a_L += 360;
	}
	
	//�����λ��Ŀ��ֵλ�ڽ����ڣ�
	if(target_yaw_a_L<=max_angle && target_yaw_a_L>=((max_angle+min_angle)/2.0))
	{
		target_yaw_a_L = max_angle;
	}
	if((target_yaw_a_L>=min_angle) && (target_yaw_a_L<=((max_angle+min_angle)/2.0)))
	{
		target_yaw_a_L = min_angle;
	}
	
	up_limit_L = min_angle - max_angle;
	low_limit_L = max_angle - min_angle;
	
	//��ǰֵλ�ڽ�����
	if(yaw_angle_L<=max_angle && yaw_angle_L>=((max_angle+min_angle)/2.0))
	{
		warning_flag_L = 1;
	}
	if((yaw_angle_L>=min_angle) && (yaw_angle_L<=((max_angle+min_angle)/2.0)))
	{
		warning_flag_L = 2;
	}
	
	//�����λ����ǰֵλ��Σ�����ڣ�
	if(((yaw_angle_L>=0) && (yaw_angle_L<=(min_angle+180))) || ((yaw_angle_L>=max_angle) && (yaw_angle_L<=0)))
	{ 
		warning_flag_L = 1; //��ǰ�Ƕ�λ���Ϸ�Σ������
	}
	else if(((yaw_angle_L>=(max_angle+180)) && (yaw_angle_L<=180)) || ((yaw_angle_L>=-180) && (yaw_angle_L<=min_angle)))
	{
		warning_flag_L = 2; //��ǰ�Ƕ�λ���·�Σ������
	}
}

void yaw_control_R(float max_angle, float min_angle)
{
	//����Сyaw��max_angle��min_angle�ڵ�����Ϊ���� ���������λ�����max_angle��min_angle��Ϊ�����������
	//�ú�����Ҫ���pid_user.c��pid_cal_yaw_a����ʹ��
	
	warning_flag_R = 0;
	
	//Խ�紦��
	if(target_yaw_a_R>180)
	{
		target_yaw_a_R -= 360;
	}
	if(target_yaw_a_R<-180)
	{
		target_yaw_a_R += 360;
	}
	
	//�����λ��Ŀ��ֵλ�ڽ����ڣ�
	if(target_yaw_a_R>=min_angle && target_yaw_a_R<=((min_angle+max_angle)/2.0))
	{
		target_yaw_a_R = min_angle;
	}
	if(target_yaw_a_R<=max_angle && target_yaw_a_R>=((min_angle+max_angle)/2.0))
	{
		target_yaw_a_R = max_angle;
	}
	
	low_limit_R = max_angle - min_angle;
	up_limit_R = min_angle - max_angle;
	
	//��ǰֵλ�ڽ�����
	if(yaw_angle_R>=min_angle && yaw_angle_R<=((min_angle+max_angle)/2.0))
	{
		warning_flag_R = 2;
	}
	if(yaw_angle_R<=max_angle && yaw_angle_R>=((min_angle+max_angle)/2.0))
	{
		warning_flag_R = 1;
	}
	
	//�����λ����ǰֵλ��Σ�����ڣ�
	if(((yaw_angle_R>=0) && (yaw_angle_R<=min_angle)) || ((yaw_angle_R>=(max_angle-180)) && (yaw_angle_R<=0)))
	{
		warning_flag_R = 2; //��ǰ�Ƕ�λ���Ϸ�Σ������
	}
	else if(((yaw_angle_R>=max_angle) && (yaw_angle_R<=180)) || ((yaw_angle_R>=-180) && (yaw_angle_R<=(min_angle-180))))
	{
		warning_flag_R = 1; //��ǰ�Ƕ�λ���·�Σ������
	}
}

void yaw_finding_L(float max_angle, float min_angle)
{
	//��ͷѲ��
	if(rotate_flag_L == 1)
	{
		target_yaw_a_L += 0.06; //��0.06/�ȵ��ٶ�Ѳ��
		if(target_yaw_a_L >= (min_angle+360))
		{
			rotate_flag_L = 2;
		}
	}
	else if(rotate_flag_L == 2) //��ͷ��ת
	{
		target_yaw_a_L -= 0.06;
		if(target_yaw_a_L <= max_angle)
		{
			rotate_flag_L = 1;
		}
	}
}

void yaw_finding_R(float max_angle, float min_angle)
{
	//��ͷѲ��
	if(rotate_flag_R == 1)
	{
		target_yaw_a_R -= 0.06; //��0.06/�ȵ��ٶ�Ѳ��
		if(target_yaw_a_R <= (max_angle-360))
		{
			rotate_flag_R = 2;
		}
	}
	else if(rotate_flag_R == 2) //��ͷ��ת
	{
		target_yaw_a_R += 0.06;
		if(target_yaw_a_R >= min_angle)
		{
			rotate_flag_R = 1;
		}
	}
}

float motor_value(int16_t k, int16_t n)
{
	//��6020����ĽǶ��Գ�ʼ�Ƕ�Ϊ0��ӳ�䵽0~+-180�� (k:�趨�ĳ�ʼ�Ƕȡ�0�� �� n:��Ҫӳ��ĽǶ�)
	//�ý���Ϊ��װ6020��ӳ�� ����װ6020��Ҫ���������ӡ�-������������
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
//************************************************************************************************************//
