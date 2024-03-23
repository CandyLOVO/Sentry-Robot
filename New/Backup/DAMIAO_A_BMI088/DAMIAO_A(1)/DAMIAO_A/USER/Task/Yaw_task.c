#include "Yaw_task.h"
#include "imu_temp_control_task.h"
//================================================YAW������������================================================//

//	���ļ�������̨�������񣬻���һ����̨�ϵ�C������ϵ�����������̨������
//	����ͷ�õ�CAN2,ID��1��2�����ID��0���ұ�ID��1
//	���Ի�����ϵ�ǵ���C���ϵ������ϵ

//================================================ȫ�ֱ���================================================//
float target_yaw_left;	//�����Դ���Ŀ��yaw��������꣩
float target_yaw_right;
float target_yaw_remote_left; //ң�����ṩ��Ŀ��ֵ
float target_yaw_remote_right;
float target_yaw_middle ; //9025���ת����Ŀ��ֵ

//��Ҫ�޸Ķ�Ӧ����ֵ�����ݰ�װ���ȡ�ĵ������ֵ�޸�
int16_t Init_encoder_left = 7912;		//���Դ���������ǰ����ʼֵ(��װ�ú�ֵ�̶�)
int16_t Init_encoder_right = 3580;		//���Դ�
uint16_t Init_encoder_middle = 26028; //һ����̨,��ǰ��Ҫ�͵���C����ǰ������һ��

float Yaw_middle_c;	//һ����̨yaw(ֻ�о�������) 9025ת��Ϊ0~+-180��ı���ֵ
//��ʱ�룺0~180,-180~0
float Yaw_value = 0;
float Yaw_left;	//����ʱ�����Դ���yaw��������꣩ ����ֵת��Ϊ0~+-180��ı���ֵ
float Yaw_right;	//����ֵת��Ϊ0~+-180��ı���ֵ
float Yaw_left_c;	//����ʱ�����Դ���yaw���������꣩ ���������IMU������ĽǶ�ֵ
float Yaw_right_c; //���������������ĽǶ�ֵ
float angle[2];

extern fp32 gyro[3];

int8_t get_flag = 0; //����ͷ��ûʶ��0����ͷʶ��1����ͷʶ��2
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
	Start_MF_send(motor_info_can_2[7].can_id);//��ʼ��9025 �������
	
	Yaw_init(); //PID������ʼ��
	
	osDelay(10);
	
	//ѭ����������
  for(;;)
  {
		Yaw_loop_init();//ѭ����ʼ������������ٶȻ�����Ϊ0
		
		//�����������ֵת��
		for(int i=0;i<2;i++)
		{
			angle[i] = rcLfFiter(motor_info_can_2[i].rotor_angle, motor_info_can_2[i].last_angle);
		}
		Yaw_read_imu();//��ȡImu�Ƕ�
		
		//ģʽѡ�񣬼���Ŀ��ֵ
		Yaw_mode_judge();//ģʽѡ��
		Yaw_target_restrict();//Ŀ��Ƕ�����(Ŀ��ǶȽ�������ʱ��������ϵ��ʼ��ʱר��)
		
		//PID
		Yaw_speed_calc();//�ǶȻ����㣨������ԽǶ����ƴ���->�ٶȻ�����ֵ
		Yaw_voltage_calc();//��ѹ�����㣨�ٶȻ���
		
		//CAN����
		Yaw_can_send();//����6020
		Voltage_Control_MF();//����������
		Current_Control_MF_send(motor_info_can_2[7].can_id,motor_info_can_2[7].set_voltage);//����9025
		for(int i=0;i<2;i++)
		{
			motor_info_can_2[i].last_angle = motor_info_can_2[i].rotor_angle;
		}
    osDelay(1);
  }

}

//================================================YAW��PID������Ŀ��IMU��ʼ��================================================//
static void Yaw_init()
{
	pid_init(&motor_pid_can_2[7],6000,2,0,72000,72000); //9025����ٶȻ�
	pid_init(&motor_pid_sita_can_2[7],3,0,50,72000,72000); //9025����ǶȻ�
	
//�ϳ���PID Ӳ��һ��
//	pid_init(&motor_pid_can_2[0],200,0.01,0,30000,30000); //��ͷ�ٶȻ�
//	pid_init(&motor_pid_sita_can_2[0],25,0,10,30000,30000); //��ͷ�ǶȻ�
//	
//	pid_init(&motor_pid_can_2[1],200,0.01,0,30000,30000); //��ͷ�ٶȻ�
//	pid_init(&motor_pid_sita_can_2[1],25,0,10,30000,30000); //��ͷ�ǶȻ�
	
//������PID
	pid_init(&motor_pid_can_2[0],150,0.01,0,30000,30000); //��ͷ�ٶȻ�
	pid_init(&motor_pid_sita_can_2[0],10,0,5,30000,30000); //��ͷ�ǶȻ�
	
	pid_init(&motor_pid_can_2[1],150,0.01,0,30000,30000); //��ͷ�ٶȻ�
	pid_init(&motor_pid_sita_can_2[1],10,0,5,30000,30000); //��ͷ�ǶȻ�
	
	Encoder_MF_read(motor_info_can_2[7].can_id);//9025��ȡ��ǰ������ֵ
	Yaw_value = MF_value(Init_encoder_middle , motor_info_can_2[7].rotor_angle , 65535); //��9025����ֵת����-180~0��0~180
	
	Yaw_left = -motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle); //��6020����ֵת����-180~0��0~180
	Yaw_right = -motor_value(Init_encoder_right,motor_info_can_2[1].rotor_angle);
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
	
	//�����������ֵת����0~+-180
	Yaw_value = MF_value(Init_encoder_middle , motor_info_can_2[7].rotor_angle , 65535); //��9025����ֵת����-180~0��0~180
	Yaw_left = -motor_value(Init_encoder_left,angle[0]);
	Yaw_right = -motor_value(Init_encoder_right,angle[1]);
	
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
		if(rc_ctrl.rc.ch[2] >= -660 && rc_ctrl.rc.ch[2]<= 660)
		{			
  		target_yaw_remote_left += rc_ctrl.rc.ch[2]/660.0 * Yaw_sita_weight; 	
			target_yaw_left = target_yaw_remote_left;
		}
		if(rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0]<= 660)
		{
			target_yaw_remote_right += rc_ctrl.rc.ch[0]/660.0 * Yaw_sita_weight;
			target_yaw_right = target_yaw_remote_right;
		}
}

//================================================Ѳ��ģʽ================================================//
static void Yaw_mode_searching()
{
	if(Sentry.L_Flag_yaw_direction == 1)
	{
		target_yaw_remote_left+=0.3;
		if(target_yaw_remote_left>=10)
		{
			Sentry.L_Flag_yaw_direction=2;
			target_yaw_remote_left-=0.3;
		}
	}
	else if(Sentry.L_Flag_yaw_direction == 2)
	{
		target_yaw_remote_left-=0.3;
		if(target_yaw_remote_left<=-190)
		{
			Sentry.L_Flag_yaw_direction=1;
			target_yaw_remote_left+=0.3;
		}		
	}
	
	if(Sentry.R_Flag_yaw_direction == 1)
	{
		target_yaw_remote_right-=0.3;
		if(target_yaw_remote_right<=-10)
		{
			Sentry.R_Flag_yaw_direction=2; //��������ת
			target_yaw_remote_right+=0.3;
		}
	}
	else if(Sentry.R_Flag_yaw_direction == 2)
	{
		target_yaw_remote_right+=0.3;
		if(target_yaw_remote_right>=190)
		{
			Sentry.R_Flag_yaw_direction=1;
			target_yaw_remote_right-=0.3;
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
	if(target_yaw_remote_left>10)
	{
		target_yaw_remote_left=10; 
		target_yaw_left=target_yaw_remote_left;
	}
	else if(target_yaw_remote_left<-180)
	{
		if(target_yaw_remote_left<-190)
		{
			target_yaw_remote_left=-190;
		}
		target_yaw_left=target_yaw_remote_left+360; 
	}
	
	if(target_yaw_remote_right<-10)
	{
		target_yaw_remote_right=-10; 
		target_yaw_right=target_yaw_remote_right;
	}
	else if(target_yaw_remote_right>180)
	{
		if(target_yaw_remote_right>190)
		{
			target_yaw_remote_right=190;
		}
		target_yaw_right=target_yaw_remote_right-360; 
	}
}

//================================================6020Ŀ��Ƕ�����===============================================//
static void Yaw_target_restrict()
{
	//����Ŀ��Ƕ�
	if(target_yaw_left>10 && target_yaw_left<170)
	{
		target_yaw_left=0;
	}
	if(target_yaw_right<-10 && target_yaw_right>-170)
	{
		target_yaw_right=0;
	}
}


//================================================�ٶȻ�������㣨��װ6020ȡ��ֵ��================================================//
static void Yaw_speed_calc()
{
	target_speed_can_2[0] +=  pid_calc_sita_span_left(&motor_pid_sita_can_2[0], target_yaw_left, Yaw_left);
	target_speed_can_2[1] +=  pid_calc_sita_span_right(&motor_pid_sita_can_2[1], target_yaw_right, Yaw_right);
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
		if(rc_ctrl.rc.ch[0] <0){
		target_yaw_middle -= 1;}
				if(rc_ctrl.rc.ch[0] >0){
		target_yaw_middle += 1;}
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
	target_speed_can_2[7] = pid_calc_sita_span(&motor_pid_sita_can_2[7], target_yaw_middle, Yaw_middle_c);  //Yaw_middle_c->IMU -180~+180
	motor_info_can_2[7].set_voltage = pid_calc(&motor_pid_can_2[7], target_speed_can_2[7],(9.55f * gyro[2])); //������yaw�Ľ��ٶ�
//	motor_info_can_2[7].set_voltage = pid_calc(&motor_pid_can_2[7], target_speed_can_2[7],motor_info_can_2[7].rotor_speed);
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
		target_yaw_left = target_yaw_remote_left;
		target_yaw_right = target_yaw_remote_right;
		Yaw_remote_restrict(); //ң������������[-20,200] ������������ֵ�������Ŀ��ֵ
	}
	else if(Sentry.Remote_mode==13) //��˿�����ͷ���Ҹ˿�����ͷ��Сyaw���λ�ò���
	{
		target_yaw_middle=0;
		Yaw_mode_remote_site();//λ�ÿ���ģʽ
		Yaw_remote_restrict();//ң���������µ�Ŀ��Ƕ����� ������������ֵ�������Ŀ��ֵ
	}
	
	
	else if(Sentry.Remote_mode==22 || Sentry.Remote_mode==23)	//�ϳ�ģʽ
	{
		//����ͷ��û��ʶ��
		if(vision_receive.L_tracking == 0 && vision_receive.R_tracking == 0)
		{
			get_flag = 0;
			Searching_Control_MF(); //9025Ѳ��0.05
			Yaw_mode_searching(); //ִ��һ��Сyaw��/��ת0.09��
			target_yaw_left = target_yaw_remote_left;
			target_yaw_right = target_yaw_remote_right;
		}
		
		//��ͷʶ��
		else if(vision_receive.L_tracking == 1 && vision_receive.R_tracking == 0)
		{
			get_flag = 1; //��ͷʶ��
			target_yaw_middle = vision_receive.yaw_L; //��ͷ�Ӿ�������yaw��ֵ
			target_yaw_left = vision_receive.L_chase_yaw - Yaw_middle_c; //��ͷСyaw���Ŀ��Ƕ�
			target_yaw_right = 0; //��ͷת�����yawƽ��
		}
		
		//��ͷʶ��
		else if(vision_receive.L_tracking == 0 && vision_receive.R_tracking == 1)
		{
			get_flag = 2; //��ͷʶ��
			target_yaw_middle = vision_receive.yaw_R; //��ͷ�Ӿ�������yaw��ֵ
			target_yaw_right = vision_receive.R_chase_yaw - Yaw_middle_c; //��ͷСyaw���Ŀ��Ƕ�
			target_yaw_left = 0; //��ͷת�����yawƽ��
		}
		
		else if(vision_receive.L_tracking == 1 && vision_receive.R_tracking == 1)
		{
			if(get_flag == 0 || get_flag == 1)
			{
				target_yaw_middle = vision_receive.yaw_L; //��ͷ�Ӿ�������yaw��ֵ
				target_yaw_left = vision_receive.L_chase_yaw - Yaw_middle_c; //��ͷСyaw���Ŀ��Ƕ�
				target_yaw_right = vision_receive.R_chase_yaw - Yaw_middle_c; //��ͷСyaw���Ŀ��Ƕ�
			}
			else if(get_flag == 2)
			{
				target_yaw_middle = vision_receive.yaw_R; //��ͷ�Ӿ�������yaw��ֵ
				target_yaw_right = vision_receive.R_chase_yaw - Yaw_middle_c; //��ͷСyaw���Ŀ��Ƕ�
				target_yaw_left = vision_receive.L_chase_yaw - Yaw_middle_c; //��ͷСyaw���Ŀ��Ƕ�
			}
		}
		//Խ�紦��
		if(target_yaw_left>180)
			target_yaw_left-=360;
		else if(target_yaw_left<-180)
			target_yaw_left+=360;
			
		if(target_yaw_right>180)
			target_yaw_right-=360;
		else if(target_yaw_right<-180)
			target_yaw_right+=360;
	}
}

//================================================Mode3������Ӧʱ����ƫ���(��λͳһΪm���Ƕ���)===============================================//
float Delta_calc(float distance)
{
	float Delta = 0;
	float d = 0.1023885;
	Delta = (float)asin(d/distance) * 57.3f;
	Delta = fabs(Delta);
	return Delta;
}

//================================================================һ�׵�ͨ�˲�===============================================================//
float rcLfFiter(float pre, float val)
{
    pre=((float)val*0.4+pre*(1-0.4));
    return pre;
}
