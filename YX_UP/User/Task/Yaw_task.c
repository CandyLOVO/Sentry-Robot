#include "Yaw_task.h"
#include "Exchange_task.h"
#include "main.h"
//	˼·�������ٶȻ�������PID��������̨������̨�����ǵ�yawֵ�������ǶȻ����в���
//  ע�⣺��̨����ת�Ǹ���
//  ��һ�汾�����ң�������Ƶİ汾
//  YAW����6020�����������C��CAN_1�����IDΪ6

//	����һЩȫ�ֱ���
extern int16_t mouse_x;
int8_t yaw_choice_flag = 0;
int8_t yaw_mode = 1;
int yaw_model_flag = 1;

fp32 ins_yaw;
fp32 ins_yaw_update = 0;
fp32 Driftring_yaw = 0;
fp32 ins_pitch;
fp32 ins_row;
fp32 init_yaw;	//��סinit_yaw��ֵ
fp32 err_yaw;		//��ס�ĽǶ�
fp32 angle_weight = 1;	//�ǶȻ�->�ٶȻ���ӳ���Ȩ��

int Yaw_count = 0;

//ǰ�����Ʊ���
int16_t Rotate_w;
int16_t Rotate_W;

#define Rotate_gain 1.23f
#define Chassis_R	30.0f
#define Chassis_r 7.5f


#define valve 20		//��ֵ
#define base 1024		//ң�����Ļ���ֵ
#define base_max 1684		
#define base_min 364
#define angle_valve 1		//�Ƕ���ֵ���������Χ�ھͲ�ȥ������
#define mouse_x_valve 10
#define mouse_x_weight 0.5f
#define Yaw_minipc_valve 0.5f
#define Yaw_minipc_weight 1.75f
#define Yaw_sita_weight 0.5f 		//λ�û�Ȩ��
#define Yaw_minipc_sita_weight 0.003f	//λ�û�Ȩ��

//У��Ư�Ʊ�־λ
int8_t Update_yaw_flag = 0;

extern uint8_t TIM1_Mode;


//����һЩ����

//��ʼ��PID����
static void Yaw_init();	

//ÿ��ѭ����ʼ��
static void Yaw_loop_init();

//��ȡimu����
static void Yaw_read_imu();

//ģʽѡ��
static void Yaw_choice();

//����������̨(���У��ٶȻ�)
static void Yaw_fix();

//����������̨(���У�λ�û�)
static void Yaw_fix_sita();

//Mode_1�µĿ����㷨,ֱ�Ӷ���Ѳ���ƶ����ٶȻ����ƣ���������
static void Yaw_mode_1();

//Mode_2�µĿ����㷨���ٶȻ����ƣ���������
static void Yaw_mode_2();

//Mode_3�µĿ����㷨��λ�û����ƣ������ǣ�
static void Yaw_mode_3();

//������YawԽ�紦��
static void detel_calc();

//ǰ������
static void Yaw_Rotate();

//�����Ƶ���
static void Yaw_mouse();

//PID����ͷ���
static void Yaw_can_send();

//�����Ӿ�����(�ٶȻ�)
static void Yaw_minipc_control();

//�����Ӿ�����(λ�û�)
static void Yaw_minipc_sita_control();

//�Ӿ�����
static void Yaw_minipc_zero();

void Yaw_task(void const *pvParameters)
{
  //��ʼ������
	
	Yaw_init();
	
	//ѭ����������
  for(;;)
  {
		Yaw_loop_init();
		Yaw_read_imu();

		//ģʽ�ж�,���Ͻǿ��ؿ������·����ڱ�����
		if(rc_ctrl.rc.s[1] == 2 && ins_yaw)
		{

				if(foe_flag)	//����Ӿ���⵽Ŀ��
				{
					Yaw_Rotate();		//ǰ�����Ʋ������̴�������ת���ٶ�
					Yaw_minipc_sita_control();	//�Ӿ�����
					Yaw_fix_sita();
				}
				
				else//û��⵽��Ѳ��
				{					
					Yaw_Rotate();				
					Yaw_mode_1();			//�ڱ�����
				}
		
		}
		
		else if(rc_ctrl.rc.s[1] == 1 || rc_ctrl.rc.s[1] == 3)
		{
			Yaw_Rotate();
			Yaw_minipc_sita_control();
			Yaw_mode_3();		
		}
		detel_calc();	//Խ�紦��
		target_speed[6] +=  pid_calc_sita(&motor_pid_sita[6], init_yaw, ins_yaw);
		//motor_info[5].set_voltage = pid_calc(&motor_pid[5], target_speed[5], motor_info[5].rotor_speed);//�ٶȻ�ת������
		motor_info[6].set_voltage = pid_calc(&motor_pid[6], target_speed[6], 20 * ins_data.gyro[2]);//�������ǵĽ��ٶȣ�rad/s -> r/min��
		Yaw_can_send();
    osDelay(1);
  }

}


//��ʼ��PID����
static void Yaw_init()	
{
	//idΪcan1��5��
	pid_init(&motor_pid[6],350,0.01,0,30000,30000);
	pid_init(&motor_pid_sita[6],20,0,10,30000,30000);
	init_yaw = ins_yaw;
}


//ѭ����ʼ��
static void Yaw_loop_init()
{
	target_speed[6]=0;
}


//��ȡimu����
static void Yaw_read_imu()
{
		//�����Ƕ�ֵ��ȡ
		ins_yaw = ins_data.angle[0];
		ins_pitch = ins_data.angle[1];
		ins_row = ins_data.angle[2];

}

static void Yaw_can_send()
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x2ff;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = 0x00;	//�ȷ��߰�λ		
  tx_data[1] = 0x00;
  tx_data[2] = (motor_info[6].set_voltage>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[3] = (motor_info[6].set_voltage)&0xff;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}

static void Yaw_Rotate()
{
	Rotate_W = (Rotate_gain * Rotate_w * Chassis_r) / Chassis_R;
	target_speed[6] += Rotate_W;
}
//����������̨
static void Yaw_fix()
{
	//ң�л�������̨(һ�������)
					if(yaw_model_flag == 1)		//�ƶ���̨�����¼�ס��̨�ĳ�ʼλ�õ�ֵ
					{
						init_yaw = ins_yaw;
						yaw_model_flag = 0;
					}
						err_yaw = ins_yaw - init_yaw;		//��ʵʱ���ݼ���ʼ����
					
					//Խ�紦��,��֤ת�����򲻱�
					if(err_yaw < -180)	//	Խ��ʱ��180 -> -180
					{
						err_yaw += 360;
					}
					
					else if(err_yaw > 180)	//	Խ��ʱ��-180 -> 180
					{
						err_yaw -= 360;
					}
				
					
					//��ֵ�ж�
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_speed[6] -= err_yaw * angle_weight;
					}
					
					else
					{
						target_speed[6] = 0;
					}
				
}

static void Yaw_fix_sita()
{
					if(yaw_model_flag == 1)		//�ƶ���̨�����¼�ס��̨�ĳ�ʼλ�õ�ֵ
					{
						init_yaw = ins_yaw;
						yaw_model_flag = 0;
					}
}

static void Yaw_mouse()
{
	if(mouse_x > mouse_x_valve || mouse_x < -mouse_x_valve)
	{
		yaw_model_flag = 1;
		target_speed[6] -= (fp32)mouse_x * mouse_x_weight;
	}
}

static void Yaw_mode_1()
{
	init_yaw -= 0.02f;
	yaw_model_flag = 1;
//	if(TIM1_Mode)
//	{
//		init_yaw -= 0.03f;
//		yaw_model_flag = 1;
//	}
//	else
//	{
//		Yaw_fix_sita();
//	}
}

static void Yaw_mode_2()
{
	
		if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
	{
		Yaw_fix();
	}
	else if( (rc_ctrl.rc.ch[0] >= base+valve && rc_ctrl.rc.ch[0] <= base_max) || (e_flag) )
	{
		target_speed[6] -= 60;
		yaw_model_flag = 1;
	}
	
	else if( (rc_ctrl.rc.ch[0] >= base_min && rc_ctrl.rc.ch[0]<base - valve ) || (q_flag) )
	{
		target_speed[6] += 60;
		yaw_model_flag = 1;
	}
}

//�½��Ӵ���
static void Yaw_choice()
{
	if(r_flag)
	{
		yaw_choice_flag = 1;
	}
	
	if( (!r_flag) && (yaw_choice_flag == 1) )	
	{
		yaw_choice_flag = 0;
		if(yaw_mode == 1)
		{
			yaw_mode = 2;
		}
		else if(yaw_mode == 2)
		{
			yaw_mode = 1;
		}
	}
}

static void Yaw_minipc_control()
{
//	if((fp32)(Yaw_minipc) > Yaw_minipc_valve || (fp32)(Yaw_minipc) < -Yaw_minipc_valve)
//	{
		target_speed[6] -= ((fp32)Yaw_minipc_fp) * Yaw_minipc_weight;
//	}
}

static void Yaw_minipc_zero()
{
	
}

static void Yaw_mode_3()
{
		if(rc_ctrl.rc.ch[0] >= 324 &&rc_ctrl.rc.ch[0]<= 1684)
		{			
			init_yaw -= (rc_ctrl.rc.ch[0]-base)/660.0 * Yaw_sita_weight; 			
		}
									
		
}

static void detel_calc()
{
	if(init_yaw >360)
	{
		init_yaw -=360;
	}
	
	else if(init_yaw<0)
	{
		init_yaw += 360;
	}
}

static void Yaw_minipc_sita_control()
{
		init_yaw -= ((fp32)Yaw_minipc_fp) * Yaw_minipc_sita_weight;
}