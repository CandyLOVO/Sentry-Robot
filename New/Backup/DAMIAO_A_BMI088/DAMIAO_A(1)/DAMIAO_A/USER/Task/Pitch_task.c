#include "Pitch_task.h"

//===============================================ȫ�ֱ���================================================//
//��װ�ú����ˮƽʱ����pitch��ֵ
int16_t Init_encoder_left_gimbal = 54;		//���Դ�������ˮƽʱ��ʼֵ(��װ�ú�ֵ�̶�)
int16_t Init_encoder_right_gimbal = 52;		//���Դ�
float target_gimbal_left;	//�����Դ���Ŀ��pitch��������꣩
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

//Ѳ��ģʽ
static void Gimbal_mode_searching();

//PID���������
static void Gimbal_can_send();

//����ֵ����
static void Gimbal_voltage_calc();

//Ŀ��ֵ����
static void Gimbal_target_restrict();

//ʵ��ֵ��λ,��̬������Bug,δʹ��
static void Gimbal_imu_restrict();

//ģʽѡ��
static void Gimbal_mode_judge();
	
void Pitch_task(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	
	Gimbal_init();	//PID������ʼ��
	
	osDelay(2000);
  for(;;)
  {
		Gimbal_zero();	//���Ŀ���ٶ�ֵ����
		Gimbal_read_motor();	//��ȡ������ֵ ������ֵת��Ϊ0~+-180 ������ͷ��ֵΪ����
		Gimbal_mode_judge();  //ģʽѡ��
		Gimbal_target_restrict();	//Ŀ��ֵ���� ������λ15�ȣ�������λ22��
		Gimbal_voltage_calc();	//����ֵ����
		Gimbal_can_send();
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

//================================================��ʼ��PID����================================================//
static void Gimbal_init()	
{
	
//�ϳ���PID Ӳ��һ��
	pid_init(&motor_pid_can_2[2],200,0.1,0,30000,30000); //��ͷ�ٶȻ�  
	pid_init(&motor_pid_sita_can_2[2],30,0,100,30000,30000); //��ͷ�ǶȻ�
	
	pid_init(&motor_pid_can_2[3],200,0.1,0,30000,30000); //��ͷ�ٶȻ�
	pid_init(&motor_pid_sita_can_2[3],30,0,100,30000,30000); //��ͷ�ǶȻ�
	
//������PID
//	pid_init(&motor_pid_can_2[2],1,0,0,30000,30000); //��ͷ�ٶȻ�  
//	pid_init(&motor_pid_sita_can_2[2],1,0,0,30000,30000); //��ͷ�ǶȻ�
//	
//	pid_init(&motor_pid_can_2[3],1,0,0,30000,30000); //��ͷ�ٶȻ�
//	pid_init(&motor_pid_sita_can_2[3],1,0,0,30000,30000); //��ͷ�ǶȻ�
	
	Gimbal_read_motor();
	target_gimbal_left = 0;
	target_gimbal_right = 0;
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
	
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

//================================================Ŀ��ֵ����================================================//
//������λ40�ȣ�������λ25��
static void Gimbal_target_restrict()
{
	if(target_gimbal_left > 22)
		target_gimbal_left=22;
	else if(target_gimbal_left < -15)
		target_gimbal_left=-15;
	
	if(target_gimbal_right > 22)
		target_gimbal_right=22;
	else if(target_gimbal_right < -15)
		target_gimbal_right=-15;
}

//================================================ʵ��ֵ��λ��ע������̬���ʱ����bug��δʹ�ã�================================================//
static void Gimbal_imu_restrict()
{
	if(Gimbal_left > 22)
		target_gimbal_left=22;
	else if(Gimbal_left < -15)
		target_gimbal_left=-15;
	
	if(Gimbal_right > 22)
		target_gimbal_right=22;
	else if(Gimbal_right < -15)
		target_gimbal_right=-15;
}

//================================================����ֵ����================================================//
static void Gimbal_voltage_calc()
{
		target_speed_can_2[2] += pid_calc_sita_span(&motor_pid_sita_can_2[2], target_gimbal_left, Gimbal_left); //��ͷ�ǶȻ�
		motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed); //��ͷ�ٶȻ�
		
		target_speed_can_2[3] -= pid_calc_sita_span(&motor_pid_sita_can_2[3], target_gimbal_right, Gimbal_right); //��ͷ�ǶȻ�
		motor_info_can_2[3].set_voltage = pid_calc(&motor_pid_can_2[3], target_speed_can_2[3], motor_info_can_2[3].rotor_speed);	//��ͷ�ٶȻ�
}

//================================================�ٶ�����================================================//
static void Gimbal_zero()
{
	target_speed_can_2[2] = 0;
	target_speed_can_2[3] = 0;
}

//================================================ң����λ�û�����ģʽ================================================//
static void Gimbal_mode_control_sita()
{
		if(rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1]<= 660)
		{
			target_gimbal_left -= rc_ctrl.rc.ch[1]/660.0 * Pitch_sita_weight; 
			target_gimbal_right = target_gimbal_left;		
		}
}

//================================================Ѳ��ģʽ================================================//
static void Gimbal_mode_searching()
{
	if(Sentry.L_Flag_pitch_direction == 1)
	{
		target_gimbal_left-=0.05;
		if(target_gimbal_left<-15)
		{
			target_gimbal_left+=0.05;
			Sentry.L_Flag_pitch_direction=2;
		}
	}
	else if(Sentry.L_Flag_pitch_direction == 2)
	{
		target_gimbal_left+=0.05;
		if(target_gimbal_left>22)
		{
			target_gimbal_left-=0.05;
			Sentry.L_Flag_pitch_direction=1;
		}
	}
	
	if(Sentry.R_Flag_pitch_direction == 1)
	{
		target_gimbal_right-=0.05;
		if(target_gimbal_right<-15)
		{
			target_gimbal_right+=0.05;
			Sentry.R_Flag_pitch_direction=2;
		}
	}
	else if(Sentry.R_Flag_pitch_direction == 2)
	{
		target_gimbal_right+=0.05;
		if(target_gimbal_right>22)
		{
			target_gimbal_right-=0.05;
			Sentry.R_Flag_pitch_direction=1;
		}
	}
}
//================================================ģʽѡ��================================================//
static void Gimbal_mode_judge()
{
	if(Sentry.Remote_mode==33)
	{
		Gimbal_mode_control_sita(); //�Ҹ����¿���Pitch��
	}
	
	else if(Sentry.Remote_mode==13) //��˿�����ͷ���Ҹ˿�����ͷ
	{
		if(rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1]<= 660)
		{
			target_gimbal_left -= rc_ctrl.rc.ch[1]/660.0 * Pitch_sita_weight; 
		}
		if(rc_ctrl.rc.ch[3] >= -660 && rc_ctrl.rc.ch[3]<= 660)
		{
			target_gimbal_right -= rc_ctrl.rc.ch[3]/660.0 * Pitch_sita_weight; 
		}
	}
	
	else if(Sentry.Remote_mode==22)	//�ϳ�ģʽ
	{
		if(vision_receive.L_tracking == 0 && vision_receive.R_tracking == 0)
		{
			Gimbal_mode_searching(); //��0.05����Ѳ����[-15,22]
		}
		
		else
		{
			target_gimbal_left = vision_receive.L_chase_pitch;
			target_gimbal_right = vision_receive.R_chase_pitch;
		}
		
		
//		if(Sentry.Flag_mode==0)  //��ѰĿ��
//		{
//			Gimbal_mode_searching(); //��0.05����Ѳ����[-40,25]
//		}
//		
//		//���鴥����ע��ֻ������λһ��
//		else if(Sentry.Flag_mode==1)  //һ��ͷʶ��Ŀ�꣬Pitch��Ӧһ�Σ���ͷĿ��ֵͬʱ�ı�
//		{
//			if(Sentry.L_Flag_foe) //��ͷʶ��Ŀ��
//			{
//				target_gimbal_left=vision_receive.L_chase_pitch; //�Ӿ�������pitchֵ
//				target_gimbal_right=target_gimbal_left;
//			}
//			else if(Sentry.R_Flag_foe) //��ͷʶ��Ŀ��
//			{
//				target_gimbal_right=vision_receive.R_chase_pitch;
//				target_gimbal_left=target_gimbal_right;
//			}
//			Sentry.Flag_mode = 2;	//pitch��Ӧ�����Ϊ2����yawȥ��Ӧ
//		}
//		
//		else if(Sentry.Flag_mode==3)	//����һ��ʶ�𵽣���yaw������Сyaw����������׷��Ŀ��
//		{
//			if(Sentry.L_Flag_foe) //��ͷʶ��Ŀ��
//				target_gimbal_left=vision_receive.L_chase_pitch;
//			
//			if(Sentry.R_Flag_foe) //��ͷʶ��Ŀ��
//				target_gimbal_right=vision_receive.R_chase_pitch;
//		}
	}
}
