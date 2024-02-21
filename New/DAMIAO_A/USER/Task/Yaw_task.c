#include "Yaw_task.h"
//================================================YAW������������================================================//

//	���ļ�������̨�������񣬻���һ����̨�ϵ�C������ϵ�����������̨������
//	����ͷ�õ�CAN2,ID��1��2�����ID��0���ұ�ID��1
//	���Ի�����ϵ�ǵ���C���ϵ������ϵ

//================================================ȫ�ֱ���================================================//
float target_yaw_left;	//�����Դ���Ŀ��yaw��������꣩
float target_yaw_right;
float target_yaw_remote_left;
float target_yaw_remote_right;
float target_yaw_middle;
int16_t Init_encoder_left = 6818;		//���Դ���������ǰ����ʼֵ(��װ�ú�ֵ�̶�)
int16_t Init_encoder_right = 7154;		//���Դ�
int16_t Init_encoder_middle = 20612;		//һ����̨,��ǰ��Ҫ�͵���C����ǰ������һ��
float Yaw_middle_c;	//һ����̨yaw(ֻ�о�������)
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

//�����Դ���λ�ÿ���ģʽ
static void Yaw_mode_remote_site();

//�����Դ���Ѳ��ģʽ
static void Yaw_mode_searching();

//ң������ԽǶ�����
static void Yaw_remote_restrict();

//�����Ƕ�����
static void Yaw_target_restrict();

//�ٶȻ�����
static void Yaw_speed_calc();

//��ѹ������
static void Yaw_voltage_calc();

//ѭ����ʼ��
static void Yaw_loop_init();

//MF9025λ�ÿ���
static void Site_Control_MF();

//ģʽѡ��
static void Yaw_mode_judge();

//9025����������
static void Voltage_Control_MF();

//9025Ѳ��ģʽ
static void Searching_Control_MF();

//Mode2������Ӧʱ����ƫ���
float Delta_calc(float distance);

//================================================YAW�����������================================================//
void Yaw_task(void const *pvParameters)
{
  //������ʼ������
	osDelay(2000);//�ϵ�ȴ�IMU�����ɹ�
	motor_info_can_2[7].can_id = 1;//��ʼ��9025���ID
	Start_MF_send(motor_info_can_2[7].can_id);//��ʼ��9025
	Yaw_init();
	osDelay(10);
	
	//ѭ����������
  for(;;)
  {
		Yaw_loop_init();//ѭ����ʼ��
		Yaw_read_imu();//��ȡImu�Ƕ�
		Yaw_mode_judge();//ģʽѡ��
		Yaw_target_restrict();//Ŀ��Ƕ�����(Ŀ��ǶȽ�������ʱ��������ϵ��ʼ��ʱר��)
		Yaw_speed_calc();//�ٶȻ����㣨������ԽǶ����ƴ���
		Yaw_voltage_calc();//��ѹ������
		Yaw_can_send();//����6020
		Voltage_Control_MF();//����������
		Current_Control_MF_send(motor_info_can_2[7].can_id,motor_info_can_2[7].set_voltage);//����9025
    osDelay(1);
  }

}

//================================================YAW��PID������Ŀ��IMU��ʼ��================================================//
static void Yaw_init()
{
	pid_init(&motor_pid_can_2[7],1,0,0,2048,2048);
	pid_init(&motor_pid_sita_can_2[7],5,0.01,0,2048,2048);
	
	pid_init(&motor_pid_can_2[0],30,0.001,0,30000,30000);
	pid_init(&motor_pid_sita_can_2[0],3,0,1,30000,30000);
	pid_init(&motor_pid_can_2[1],30,0.001,0,30000,30000);
	pid_init(&motor_pid_sita_can_2[1],3,0,1,30000,30000);
	
	Encoder_MF_read(motor_info_can_2[7].can_id);//��ȡ��ǰ������ֵ
	Yaw_middle_c = MF_value(Init_encoder_middle , motor_info_can_2[7].rotor_angle , 65535);
	
	Yaw_left = motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle);
	Yaw_right = motor_value(Init_encoder_right,motor_info_can_2[1].rotor_angle);
	target_yaw_middle = Yaw_middle_c;
	target_yaw_left = Yaw_left;
	target_yaw_right = Yaw_right;

}

//================================================YAW��Ƕȶ�ȡ===============================================//
static void Yaw_read_imu()
{
	//INS_angle[0]��C�������ǵ�ֵ
	//ԭʼ����˳ʱ��Ϊ��
	//�������ʱ��Ϊ��(�������ϵ)
	//    0
	//180 -180
	
	Yaw_middle_c = MF_value(Init_encoder_middle,motor_info_can_2[7].rotor_angle , 65535);
	Yaw_left = motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle);
	Yaw_right = motor_value(Init_encoder_right,motor_info_can_2[1].rotor_angle);
	
	//��C���ϵ���һ�̵�����ϵΪ������ϵ(��������ϵ)
	Yaw_left_c = Yaw_left + Yaw_middle_c;
	Yaw_right_c = Yaw_right + Yaw_middle_c;
	//Խ�紦��
	if(Yaw_left_c>180)
		Yaw_left_c-=360;
	else if(Yaw_left_c<-180)
		Yaw_left_c+=360;
	
	if(Yaw_right_c>180)
		Yaw_right_c-=360;
	else if(Yaw_right_c<-180)
		Yaw_right_c+=360;	
}

//================================================λ�ÿ���ģʽ================================================//
static void Yaw_mode_remote_site()
{
		if(rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0]<= 660)
		{			
			target_yaw_remote_left -= rc_ctrl.rc.ch[0]/660.0 * Yaw_sita_weight; 	
			target_yaw_left = target_yaw_remote_left;
		}
		if(rc_ctrl.rc.ch[2] >= -660 && rc_ctrl.rc.ch[2]<= 660)
		{
			target_yaw_remote_right -= rc_ctrl.rc.ch[2]/660.0 * Yaw_sita_weight;
			target_yaw_right = target_yaw_remote_right;
		}
}

//================================================Ѳ��ģʽ================================================//
static void Yaw_mode_searching()
{
	if(Sentry.L_Flag_yaw_direction == 1)
	{
		target_yaw_remote_left-=0.09;
		if(target_yaw_remote_left<=-20)
		{
			Sentry.L_Flag_yaw_direction=2;
			target_yaw_remote_left+=0.09;
		}
	}
	else if(Sentry.L_Flag_yaw_direction == 2)
	{
		target_yaw_remote_left+=0.09;
		if(target_yaw_remote_left>=200)
		{
			Sentry.L_Flag_yaw_direction=1;
			target_yaw_remote_left-=0.09;
		}		
	}
	
	if(Sentry.R_Flag_yaw_direction == 1)
	{
		target_yaw_remote_right+=0.09;
		if(target_yaw_remote_right>=20)
		{
			Sentry.R_Flag_yaw_direction=2;
			target_yaw_remote_right-=0.09;
		}
	}
	else if(Sentry.R_Flag_yaw_direction == 2)
	{
		target_yaw_remote_right-=0.09;
		if(target_yaw_remote_right<=-200)
		{
			Sentry.R_Flag_yaw_direction=1;
			target_yaw_remote_right+=0.09;
		}		
	}
}

//================================================Yaw�����������================================================//
static void Yaw_can_send()
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

//================================================���ƽǶ�����================================================//
static void Yaw_remote_restrict()
{
	if(target_yaw_remote_left<-20)
	{
		target_yaw_remote_left=-20; 
		target_yaw_left=target_yaw_remote_left;
	}
	else if(target_yaw_remote_left>180)
	{
		if(target_yaw_remote_left>200)
		{
			target_yaw_remote_left=200;
		}
		target_yaw_left=target_yaw_remote_left-360; 
	}
	
	if(target_yaw_remote_right>20)
	{
		target_yaw_remote_right=20; 
		target_yaw_right=target_yaw_remote_right;
	}
	else if(target_yaw_remote_right<-180)
	{
		if(target_yaw_remote_right<-200)
		{
			target_yaw_remote_right=-200;
		}
		target_yaw_right=target_yaw_remote_right+360; 
	}
}

//================================================6020Ŀ��Ƕ�����===============================================//
static void Yaw_target_restrict()
{
	//����Ŀ��Ƕ�
	if(target_yaw_left<-20 && target_yaw_left>-160)
	{
		target_yaw_left=0;
	}
	if(target_yaw_right>20 && target_yaw_right<160)
	{
		target_yaw_right=0;
	}
}


//================================================�ٶȻ�������㣨��װ6020ȡ��ֵ��================================================//
static void Yaw_speed_calc()
{
	target_speed_can_2[0] -=  pid_calc_sita_span_left(&motor_pid_sita_can_2[0], target_yaw_left, Yaw_left);
	target_speed_can_2[1] -=  pid_calc_sita_span_right(&motor_pid_sita_can_2[1], target_yaw_right, Yaw_right);
}

//================================================��ѹ������================================================//
static void Yaw_voltage_calc()
{
	motor_info_can_2[0].set_voltage = pid_calc(&motor_pid_can_2[0], target_speed_can_2[0], motor_info_can_2[0].rotor_speed);
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[1].rotor_speed);
}

//================================================ѭ����ʼ��================================================//
static void Yaw_loop_init()
{
	target_speed_can_2[7] = 0;
	target_speed_can_2[0] = 0;
	target_speed_can_2[1] = 0;
}
	
//================================================MF9025λ�ÿ���===============================================//
static void Site_Control_MF()
{
	if(rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0]<= 660)
	{
		target_yaw_middle -= rc_ctrl.rc.ch[0]/660.0 * Yaw_sita_weight;
		if(target_yaw_middle > 180)
		{
			target_yaw_middle -= 360;
		}
		else if(target_yaw_middle < -180)
		{
			target_yaw_middle += 360;
		}
	}
}

//================================================MF9025����������(���������ƺ���)===============================================//
static void Voltage_Control_MF()
{
	target_speed_can_2[7] -= pid_calc_sita_span(&motor_pid_sita_can_2[7], target_yaw_middle, Yaw_middle_c);
	motor_info_can_2[7].set_voltage = pid_calc(&motor_pid_can_2[7], target_speed_can_2[7],motor_info_can_2[7].rotor_speed);
	motor_info_can_2[7].set_voltage = Current_Limit_MF(motor_info_can_2[7].set_voltage);//���õ������ƺ���
}

//================================================9025Ѳ��ģʽ===============================================//
static void Searching_Control_MF()
{
	target_yaw_middle+=0.1;
	if(target_yaw_middle>180)
	{
		target_yaw_middle-=360;
	}
}

//================================================Yaw����ģʽ��ת===============================================//
static void Yaw_mode_judge()
{
	if(Sentry.Remote_mode==33)
	{
		Site_Control_MF();//MF9025λ��ģʽ(ң����)
		target_yaw_remote_left = 0;
		target_yaw_remote_right = 0;
		Yaw_remote_restrict();
	}
	else if(Sentry.Remote_mode==13)
	{
		target_yaw_middle=0;
		Yaw_mode_remote_site();//λ�ÿ���ģʽ
		Yaw_remote_restrict();//ң���������µ�Ŀ��Ƕ�����
	}
	
	
	else if(Sentry.Remote_mode==22)	//�ϳ�ģʽ
	{
		if(Sentry.Flag_mode==0)  //��ѰĿ��
		{
			Searching_Control_MF();
			Yaw_mode_searching();
			Yaw_remote_restrict();
		}
		else if(Sentry.Flag_mode==1)  //ʶ��Ŀ��ȴ���һ����Ӧ
		{
			float Delta;	//�涨��һֱ�Ǹ�����
			if(Sentry.L_Flag_foe)
			{
				target_yaw_middle=vision.L_yaw;
				Delta_calc(vision_receive.L_distance);
			}
			else if(Sentry.R_Flag_foe)
			{
				target_yaw_middle=vision.R_yaw;
				Delta_calc(vision_receive.R_distance);
			}
			target_yaw_left = -Delta;
			target_yaw_right = Delta;
			
			target_yaw_remote_left = -Delta;	//ˢ��Ѳ����ʼֵ���ָ�Ѳ��ʱ��˿��
			target_yaw_remote_right = Delta;
			
			osDelay(1);//��pitchʱ����Ӧһ��
			Sentry.Flag_mode = 2;  //��Ӧһ�ξ���λ
		}
		else if(Sentry.Flag_mode==2)  //�������ϵ���Сyaw��̬����Ŀ��
		{
			if(Sentry.L_Flag_foe)
				target_yaw_left = vision.L_yaw - Yaw_middle_c;
			if(Sentry.R_Flag_foe)
				target_yaw_right = vision.R_yaw - Yaw_middle_c;
			//Խ�紦��
			if(target_yaw_left>180)
				target_yaw_left-=360;
			else if(target_yaw_left<-180)
				target_yaw_left+=360;
			if(target_yaw_right>180)
				target_yaw_right-=360;
			else if(target_yaw_right<-180)
				target_yaw_right+=360;
			
			//ͷ����λ���ô�Yaw
			if((target_yaw_left<-20 && target_yaw_left>-160) || (target_yaw_right>20 && target_yaw_right<160))
			{
				Sentry.Flag_mode = 1;
			}
			
			//����ʧĿ��ָ�Ѳ��ģʽ
			if(Sentry.L_Flag_foe == 0 && Sentry.R_Flag_foe == 0)
			{
				Sentry.Flag_mode = 0;
			}
		}
	}
}

//================================================Mode2������Ӧʱ����ƫ���(��λͳһΪm���Ƕ���)===============================================//
float Delta_calc(float distance)
{
	float Delta = 0;
	float d = 0.1023885;
	Delta = (float)asin(d/distance) * 57.3f;
	Delta = fabs(Delta);
	return Delta;
}