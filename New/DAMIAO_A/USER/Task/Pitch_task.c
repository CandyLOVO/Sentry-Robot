#include "Pitch_task.h"

//===============================================ȫ�ֱ���================================================//
int16_t Init_encoder_left_gimbal = 6818;		//���Դ�������ˮƽʱ��ʼֵ(��װ�ú�ֵ�̶�)
int16_t Init_encoder_right_gimbal = 7154;		//���Դ�
float target_gimbal_left;	//�����Դ���Ŀ��yaw��������꣩
float target_gimbal_right;
float Gimbal_left;
float Gimbal_right;	

//================================================����================================================//

//��ʼ��PID����
static void Gimbal_init();	

//��������
static void Gimbal_zero();

//���㵱ǰ��������ֵ
static void Gimbal_read_motor();

//ң��������ģʽ(λ�û�)
static void Gimbal_mode_control_sita();

//Ѳ��ģʽ(λ�û�)
//static void gimbal_mode_search_sita();

//PID���������
static void Gimbal_can_send();

//����ֵ����
static void Gimbal_voltage_calc();

//Ŀ��ֵ����
static void Gimbal_target_restrict();

//ʵ��ֵ��λ
static void Gimbal_imu_restrict();

//��������(λ�û�)
static void Gimbal_minipc_control_sita();

// pitch

void Pitch_task(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	Gimbal_init();	//PID������ʼ��
	osDelay(2000);
  for(;;)
  {
		Gimbal_zero();	//�ٶ�����
		Gimbal_read_motor();	//��ȡ������ֵ
		Gimbal_minipc_control_sita();	//λ�û��Ӿ���׼
		Gimbal_mode_control_sita();	//ң����λ�û�����ģʽ
		Gimbal_target_restrict();	//Ŀ��ֵ����
//		Gimbal_imu_restrict();	//ʵ��ֵ��λ
		Gimbal_voltage_calc();	//����ֵ����
		Gimbal_can_send();
//		else if(rc_ctrl.rc.s[1]==2)		//�ϳ�ģʽ
//		{
//			if(Sentry.foe_flag)	//�����⵽Ŀ��
//			{
//				gimbal_minipc_control_sita(); //�Ӿ���׼
//			}			
//			else
//			{
//				gimbal_mode_search_sita();	//�ڱ�Ѳ��ģʽ
//			}
//		}
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

//================================================��ʼ��PID����================================================//
static void Gimbal_init()	
{
	
	pid_init(&motor_pid_can_2[2],30,0.001,0,30000,30000);
	pid_init(&motor_pid_sita_can_2[2],3,0,1,30000,30000);
	pid_init(&motor_pid_can_2[3],30,0.001,0,30000,30000);
	pid_init(&motor_pid_sita_can_2[3],3,0,1,30000,30000);
	Gimbal_read_motor();
	target_gimbal_left = Gimbal_left;
	target_gimbal_right = Gimbal_right;
} 

//================================================���㵱ǰ������ֵ================================================//
//ע���;��ڱ���������ȣ���Ҫ��ͷΪ������������2��6020������Ϊһ��һ��
static void Gimbal_read_motor()
{
	Gimbal_left = -motor_value(Init_encoder_left_gimbal,motor_info_can_2[2].rotor_angle);
	Gimbal_right = motor_value(Init_encoder_right_gimbal,motor_info_can_2[3].rotor_angle);
}

//================================================Pitch���ݷ���================================================//
static void Gimbal_can_send()
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x1ff;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (motor_info_can_2[0].set_voltage>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[1] = (motor_info_can_2[0].set_voltage)&0xff;
  tx_data[2] = (motor_info_can_2[1].set_voltage>>8)&0xff;	
  tx_data[3] = (motor_info_can_2[1].set_voltage)&0xff;
  tx_data[4] = (motor_info_can_2[2].set_voltage>>8)&0xff;
  tx_data[5] = (motor_info_can_2[2].set_voltage)&0xff;
  tx_data[6] = (motor_info_can_2[3].set_voltage>>8)&0xff;
  tx_data[7] = (motor_info_can_2[3].set_voltage)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);
}

//================================================Ŀ��ֵ����================================================//
//������λ40�ȣ�������λ25��
static void Gimbal_target_restrict()
{
	if(target_gimbal_left > 25)
		target_gimbal_left=25;
	else if(target_gimbal_left < -40)
		target_gimbal_left=-40;
	
	if(target_gimbal_right > 25)
		target_gimbal_right=25;
	else if(target_gimbal_right < -40)
		target_gimbal_right=-40;
}

//================================================ʵ��ֵ��λ================================================//
static void Gimbal_imu_restrict()
{
	if(Gimbal_left > 25)
		target_gimbal_left=20;
	else if(Gimbal_left < -40)
		target_gimbal_left=-35;
	
	if(Gimbal_right > 25)
		target_gimbal_right=20;
	else if(Gimbal_right < -40)
		target_gimbal_right=-35;
}

//================================================����ֵ����================================================//
static void Gimbal_voltage_calc()
{
		target_speed_can_2[2] += pid_calc_sita_span(&motor_pid_sita_can_2[2], target_gimbal_left, Gimbal_left);
		target_speed_can_2[3] -= pid_calc_sita_span(&motor_pid_sita_can_2[3], target_gimbal_right, Gimbal_right);
		motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed);
		motor_info_can_2[3].set_voltage = pid_calc(&motor_pid_can_2[3], target_speed_can_2[3], motor_info_can_2[3].rotor_speed);	
}

//================================================�ٶ�����================================================//
static void Gimbal_zero()
{
	target_speed_can_2[2] = 0;
	target_speed_can_2[3] = 0;
}

//================================================ң����λ�û�����ģʽ(����������Ȩ��)================================================//
static void Gimbal_mode_control_sita()
{
		if(rc_ctrl.rc.ch[3] >= -660 && rc_ctrl.rc.ch[3]<= 660)
		{
			target_gimbal_left -= (rc_ctrl.rc.ch[3])/660.0 * Pitch_sita_weight; 
			target_gimbal_right = target_gimbal_left;		
		}
}

//================================================�Ӿ���׼(λ�û�ģʽ)================================================//
static void Gimbal_minipc_control_sita()
{
//	target_pitch =chase.pitch;
}

//================================================Ѳ��ģʽ(λ�û�ģʽ)================================================//
//static void gimbal_mode_search_sita()
//{
//	switch(TIM1_Mode)
//	{
//		case 1: target_pitch -= 0.03f;break;
//		case 2: target_pitch += 0.03f;break;
//	}
//}