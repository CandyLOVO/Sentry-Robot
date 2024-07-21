#include "Yaw_Task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "MG5010_control.h"
#include "rc_potocal.h"
#include "pid_user.h"
#include "Exchange_Task.h"
#include "judge.h"

pidTypeDef pid_5010_s;
pidTypeDef pid_5010_a;
uint8_t can_send_data_5010[8];
int32_t initial_angle; //5010��������������򡿵ĳ�ʼ����ֵ
float yaw_angle; //��yaw5010��ǰ�Ƕȣ�0~+-180��
float target_angle_5010;
float target_speed_5010;
float output_5010;
int16_t heart_direction[4]; //�ܻ���װ�װ�
float last_target_angle_5010; //��һ֡Ŀ��ֵ
uint16_t time_delay = 0; //��ס��ʱ
uint32_t time_delay_heart = 0;
uint8_t flag_suo = 0; //��׼����̨��ס
uint8_t flag_heart = 0;
uint8_t last_id;
float tar;

extern RC_ctrl_t rc_ctrl;
extern motor_5010_info motor_5010;
extern double yaw12; //��̨������yawֵ
extern float yaw_From_L; //�Ӿ�������Ŀ��yawֵ
extern float yaw_gyro; //��̨������yaw���ٶ�ֵ
extern uint8_t L_tracking;
extern uint8_t M_tracking;
extern int8_t flag;
extern TIM_HandleTypeDef htim4;
extern Sentry_t Sentry;
extern Rx_naving Rx_nav;
extern fp32 error_theta; //��̨����ϵ���������ϵ��н�(��ʱΪ0~360��) ���ڽ��պ���Ҫ������theta���д���

void Yaw_task(void const * argument)
{
	MG5010_init(); //����5010���
	yaw_init(); //��ʼ��5010�������ҪУ׼��yaw���������ֵ��
	flag_suo = 0;
	time_delay=1001;
	osDelay(3000);
	
  for(;;)
  {
		if(flag == 1)
		{
		yaw_angle = -motor_value(initial_angle, motor_5010.angle, 65535); //��5010����ֵת��Ϊ0~+-180����������ͷ������תΪ-������תΪ+��
		
		//ң��������ģʽ����->�м䣬��->�м�            ���Ե��̸�����̨ģʽ����->���ϣ���->�м�
		if((rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==3) || (rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==1))
		{
			yaw_control();
			if(Rx_nav.poing == 1) //��������������ָ��
			{
				tar = Rx_nav.yaw_target; //����yawת���µķ���
			}
		}
		
		//�����ϳ�ģʽ����->���£���->����
		if(rc_ctrl.rc.s[0]==2 && rc_ctrl.rc.s[1]==2)
		{
			if(Rx_nav.poing == 1) //��������������ָ��
			{
				target_angle_5010 = Rx_nav.yaw_target; //����yawת���µķ���
			}
			else //��������
			{
				//�ĸ�����ͷ��û��ʶ��
				if(L_tracking==0 && Rx_nav.R_tracking==0 && M_tracking==0)
				{
					//��yaw��׼��ʱ
					if((flag_suo == 2)&&(time_delay <= 1000)) //��һ��״̬Ϊ��ס������1000ms�ڣ�
					{
						target_angle_5010 = last_target_angle_5010; //Ŀ��Ƕ�Ϊ��סʱ�ĽǶ�
						time_delay++;
					}
					//����Ѳ��
					else
					{
						//װ�װ��ܻ���
						if(Sentry.hurt_type==0 && Sentry.armor_id != last_id)
						{
							flag_heart = 1;
							last_id = Sentry.armor_id;
							target_angle_5010 = heart_direction[Sentry.armor_id] + yaw12 - error_theta*180/3.14; //װ�װ��ܻ���λ
							if(target_angle_5010 > 180)
							{
								target_angle_5010 -= 360;
							}
							if(target_angle_5010 < -180)
							{
								target_angle_5010 += 360;
							}
							last_target_angle_5010 = target_angle_5010; //����ô���ס��Ŀ��ֵ
							time_delay = 0; //��ʼ����ʱ����ֵ
							flag_suo = 1; //��ס��־λ
						}
						//װ�װ�û���յ�����
						else if(Rx_nav.Flag_turn == 1)
						{
							flag_heart = 0;
							flag_suo = 2; //δ��ס��־λ
							yaw_finding(); //��yawѲ��
						}
					}	
				}
				//������һ������ͷʶ��
				else if(L_tracking==1 || Rx_nav.R_tracking==1 || M_tracking==1)
				{
					time_delay = 0; //��ʼ����ʱ����ֵ
					flag_suo = 1; //��ס��־λ
					
					if(L_tracking==1 && Rx_nav.R_tracking==1) //����ͷ����ס
					{
						if((yaw_From_L-yaw12)<20 && (yaw_From_L-yaw12)>-20) //ת���Ƕ�С����ֵ
						{
							target_angle_5010 = target_angle_5010;
						}
						else
						{
							target_angle_5010 = yaw_From_L;
						}
					}
					else if(L_tracking==1 && Rx_nav.R_tracking==0) //��ͷ��ס
					{
						if((yaw_From_L-yaw12)<20 && (yaw_From_L-yaw12)>-20) //ת���Ƕ�С����ֵ
						{
							target_angle_5010 = target_angle_5010;
						}
						else
						{
							target_angle_5010 = yaw_From_L;
						}
					}
					else if(L_tracking==0 && Rx_nav.R_tracking==1) //��ͷ��ס
					{
						if((Rx_nav.yaw_From_R-yaw12)<20 && (Rx_nav.yaw_From_R-yaw12)>-20) //ת���Ƕ�С����ֵ
						{
							target_angle_5010 = target_angle_5010;
						}
						else
						{
							target_angle_5010 = Rx_nav.yaw_From_R;
						}
					}
					else if(L_tracking==0 && Rx_nav.R_tracking==0 && M_tracking==1)//�м��ͷ��ס
					{
						target_angle_5010 = yaw_From_L;
					}
					last_target_angle_5010 = target_angle_5010; //����ô���ס��Ŀ��ֵ
				}
			}
		}
		
		//PID����
		target_speed_5010 = pid_I_control(&pid_5010_a, yaw12, target_angle_5010);
		output_5010 = pid_cal_s(&pid_5010_s, (9.55f*yaw_gyro), target_speed_5010);
		//CAN2���ݷ���
		speed_control_send(output_5010);
		can_cmd_send_5010(can_send_data_5010);
		}
    osDelay(1);
  }
}

/*********************************************************����ʵ��*********************************************************/
void yaw_init(void)
{
	//��yaw5010��ֵ��ʼ��
	initial_angle = 56763; //ͷ�������������ʱ�ı���ֵ
	target_angle_5010 = 0;
	target_speed_5010 = 0;
	pid_init(&pid_5010_s,25000,0,0,700000,700000); //PID��ʼ�� PI
	pid_init(&pid_5010_a,1,0,0,700000,700000); //PD
//	pid_init(&pid_5010_s,7000,2,0,700000,700000); //PID??? PI
//	pid_init(&pid_5010_a,4,0,23,700000,700000); //PD
	
	heart_direction[0] = 0;
	heart_direction[1] = 90;
	heart_direction[2] = 180;
	heart_direction[3] = -90;
}

void yaw_control(void)
{
	target_angle_5010 += rc_ctrl.rc.ch[2]*0.1/660;
	if(target_angle_5010 > 180)
	{
		target_angle_5010 -= 360;
	}
	else if(target_angle_5010 < -180)
	{
		target_angle_5010 += 360;
	}
}

void yaw_finding(void)
{
	//��yawѲ��
	target_angle_5010 += 0.03;
	if(target_angle_5010 > 180)
	{
		target_angle_5010 -= 360;
	}
	else if(target_angle_5010 < -180)
	{
		target_angle_5010 += 360;
	}
}

float motor_value(int32_t k, int32_t n, int32_t max)
{
	//��MG5010���(ͨ�ð�)�ĽǶ��Գ�ʼ�Ƕ�Ϊ0��ӳ�䵽0~+-180�� (k:�趨�ĳ�ʼ�Ƕȡ�0�� �� n:��Ҫӳ��ĽǶ�) �õ����ǵ�װ���ת����0~+-180
	//��3�������Ǳ��������ֵ
	int32_t middle = (max+1)/2;
	float output;
	if(k>=0 && k<middle){
		if(n>=0 && n<(k+middle)){
			n = k - n;
			output = (float)n * 360.f / (float)max;
			return output;
		}
		else if(n>=(k+middle) && n<max){
			n = max - n + k;
			output = (float)n * 360.f / (float)max;
			return output;
		}
	}
	
	else if(k>=middle && k<max){
		if(n>=0 && n<(k-middle)){
			n = -max + k - n;
			output = (float)n * 360.f / (float)max;
			return output;
		}
		else if(n>=(k-middle) && n<max){
			n = k - n;
			output = (float)n * 360.f / (float)max;
			return output;
		}
	}
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim == (&htim4))
//  {
//		time_delay++;
//	}
//}
/**************************************************************************************************************************/
