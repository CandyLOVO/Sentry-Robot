#include "Yaw_task.h"
//================================================YAW������������================================================//

//	���ļ�������̨�������񣬻���һ����̨�ϵ�C������ϵ�����������̨������
//	����ͷ�õ�CAN2,ID��1��2�����ID��0���ұ�ID��1

//================================================ȫ�ֱ���================================================//
float target_yaw_left;	//�����Դ���Ŀ��yaw
float target_yaw_right;
int16_t Init_encoder_left = 1000;		//���Դ���������ǰ����ʼֵ(��װ�ú�ֵ�̶�)
int16_t Init_encoder_right = 2000;		//���Դ�
float Yaw_left;	//����ʱ�����Դ���yaw��������꣩
float Yaw_right;	
float Yaw_left_c;	//����ʱ�����Դ���yaw���������꣩
float Yaw_right_c;	
//================================================����================================================//

//��ʼ��PID����
static void Yaw_init();	

//ÿ��ѭ����ʼ��
static void Yaw_loop_init();

//��ȡC��IMUֵ
static void Yaw_read_imu();

//================================================YAW�����������================================================//
void Yaw_task(void const *pvParameters)
{
  //������ʼ������
	Yaw_init();
	osDelay(2000);//�ϵ�ȴ�IMU�����ɹ�
	
	//ѭ����������
  for(;;)
  {
		Yaw_read_imu();//��ȡImu�Ƕ�

    osDelay(1);
  }

}

//================================================YAW��PID������ʼ��================================================//
static void Yaw_init()
{
	pid_init(&motor_pid_can_2[0],60,0.001,0,30000,30000);
	pid_init(&motor_pid_sita_can_2[0],5,0,3,30000,30000);
	pid_init(&motor_pid_can_2[1],60,0.001,0,30000,30000);
	pid_init(&motor_pid_sita_can_2[1],5,0,3,30000,30000);
}

//================================================YAW��Ƕȶ�ȡ===============================================//
static void Yaw_read_imu()
{
	//INS_angle[0]��C�������ǵ�ֵ
	//ԭʼ����˳ʱ��Ϊ��
	//    0
	//-180 180
	//����Ϊ����ǰ
	
	//�������ʱ��Ϊ��(�������ϵ)
	int16_t Yaw_left_int = motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle);
	int16_t Yaw_right_int = motor_value(Init_encoder_left,motor_info_can_2[1].rotor_angle);
	Yaw_left = -(float)Yaw_left_int;
	Yaw_right = -(float)Yaw_right_int;
	
	//��C���ϵ���һ�̵�����ϵΪ������ϵ(��������ϵ)
	Yaw_left_c = Yaw_left + INS_angle[0];
	Yaw_right_c = Yaw_right + INS_angle[0];
	//Խ�紦��ûд����
}

//================================================λ�ÿ���ģʽ================================================//
static void Yaw_mode_remote_site()
{
//		if(rc_ctrl.rc.ch[0] >= 324 && rc_ctrl.rc.ch[0]<= 1684)
//		{			
//			target_yaw -= (rc_ctrl.rc.ch[0]-base)/660.0 * Yaw_sita_weight; 			
//		}
}