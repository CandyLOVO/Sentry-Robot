#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "INS_task.h"
#include "Pitch_task.h"
#include "Exchange_task.h"

//��һ�棺
//��������������̨����ģʽѡ�񣬿��ƣ�У׼��
//��װһЩ�����������п��Ƶ���
//Pitch������C��CAN_2�����IDΪ5
//�������⣺��Ҫ����̨Pitch�������·���������Pitch�ܸ�Ƶ���Ӱ�죬�����ٶ�̫��

//�ڶ��棺
//����˫6020�ṹ���¼ӵ�Pitch����C��CAN_2�����IDΪ6

//����һЩ����
//��λ��������е������
#define Up_inf 35
#define Down_inf 0
#define mouse_y_valve 10
#define mouse_y_weight 12.0f
#define Pitch_minipc_valve 0.5f
#define Pitch_minipc_weight	0.5f
#define Pitch_sita_weight 0.2f
#define Pitch_sita_minipc_weight 0.002f


#define Pitch_up 4500
#define Pitch_down 4025

//imu����
fp32 Err_pitch;
int16_t Up_pitch;
int16_t Down_pitch;
uint16_t Remember_pitch = 0;
uint8_t Remember_pitch_flag = 1;
extern ins_data_t ins_data;
extern int16_t mouse_y;

uint16_t gimbal_rotor_angle;
uint16_t gimbal_rotor_angle_2;
float Pitch_imu;
float Pitch_imu_speed;
float init_pitch;
float ins_pitch_speed;

//��ʱ��������
uint16_t TIM1_Count = 0;
uint8_t TIM1_Mode = 1;

//��ʼ��PID����
static void gimbal_init();	

//��ʼ����
static void gimbal_zero();

//У�����ӳɹ�
static bool gimbal_judge();	

//��ȡimu����
static void gimbal_read_imu();

//��ȡ��������ֵ
static void gimbal_read();

//ģʽѡ��
static void gimbal_choice();

//Mode_1�µĿ����㷨(�ٶȻ�)
static void gimbal_mode_1();

//Mode_2�µ�Ѳ�߿����㷨(�ٶȻ�)
static void gimbal_mode_2();

//Mode_3�µĿ����㷨(λ�û�--->imu)
static void gimbal_mode_3();

//Mode_4�µ�Ѳ�߿����㷨(λ�û�)
static void gimbal_mode_4();

//Mode_4�µĿ����㷨(λ�û�--->������)
static void gimbal_mode_5();

//PID����ͷ���
static void gimbal_can_send();

//��λ����������
static void gimbal_limit();

//��λ�������ǣ�
static void gimbal_imu_limit();

//������Pitch(����)
static void gimbal_mouse();

//��������
static void gimbal_minipc_control();

//��������(λ�û�)
static void gimbal_minipc_sita_control();

// pitch

void Pitch_task(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */

	gimbal_init();	//PID��������
	
  for(;;)
  {
		gimbal_zero();
		gimbal_read();
		gimbal_read_imu();
		if(rc_ctrl.rc.s[1]==3 || rc_ctrl.rc.s[1]==1	)
		{
			gimbal_mode_3();	
		  gimbal_minipc_sita_control();
		 }
		  
		else if(rc_ctrl.rc.s[1]==2)		//�ڱ����ԣ����Ͻǲ������¶�����
		{
			if(foe_flag)	//�����⵽Ŀ��
			{
				gimbal_minipc_sita_control(); 
			}
			
			else
			{
				gimbal_mode_4();		
			}
		}
		//gimbal_imu_limit();
//		target_speed_can_2[4] -= pid_calc_sita(&motor_pid_sita_can_2[4], init_pitch, ins_data.angle[1]);
		target_speed_can_2[5] += pid_calc_sita(&motor_pid_sita_can_2[5], init_pitch, ins_data.angle[1]);
//		target_speed_can_2[4] -= pid_calc_sita(&motor_pid_sita_can_2[4], init_pitch+3632, motor_info_can_2[4].rotor_angle);
		//target_speed_can_2[5] += pid_calc_sita(&motor_pid_sita_can_2[5], init_pitch, motor_info_can_2[5].rotor_angle);
		//motor_info_can_2[4].set_voltage = pid_calc(&motor_pid_can_2[4], target_speed_can_2[4], Pitch_imu_speed * -20);
//		if(target_speed_can_2[5] > 100)
//		{
//			target_speed_can_2[5] = 100;
//		}
//		else if(target_speed_can_2[5] < -100)
//		{
//			target_speed_can_2[5] = -100;
//		}
		ins_pitch_speed = ins_data.gyro[1] * 20;
		motor_info_can_2[5].set_voltage = pid_calc(&motor_pid_can_2[5], target_speed_can_2[5], ins_pitch_speed );
//		motor_info_can_2[4].set_voltage = -motor_info_can_2[5].set_voltage;
//		motor_info_can_2[4].set_voltage = pid_calc(&motor_pid_can_2[4], target_speed_can_2[4], motor_info_can_2[4].rotor_speed);
//		motor_info_can_2[5].set_voltage = pid_calc(&motor_pid_can_2[5], target_speed_can_2[5], motor_info_can_2[5].rotor_speed);
		gimbal_can_send();		
		
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

//��ʼ��PID����
static void gimbal_init()	
{
	//pid_init(&motor_pid_can_2[4],450,0.01,0,30000,30000);// 120 0.01 0
	pid_init(&motor_pid_can_2[5],250,0.01,0,30000,30000);// 120 0.01 0
	//pid_init(&motor_pid_sita_can_2[4],10,0,200,30000,30000);// 10 0 1300
	pid_init(&motor_pid_sita_can_2[5],5,0,100,30000,30000);// 10 0 1300
//	pid_init(&motor_pid_can_2[4],20,0,0,30000,30000);
//	pid_init(&motor_pid_can_2[5],20,0,0,30000,30000);
//	pid_init(&motor_pid_sita_can_2[4],10,0,1,30000,30000);
//	pid_init(&motor_pid_sita_can_2[5],10,0,1,30000,30000);
	init_pitch = Pitch_imu;
}


//У�����ӳɹ�
static bool gimbal_judge()
{

}


//��������ֵ
static void gimbal_read()
{
	gimbal_rotor_angle = motor_info_can_2[4].rotor_angle;	//���·�:0E30   ���Ϸ�:10B0  ����:3632->4272  ,������640
	gimbal_rotor_angle_2 = motor_info_can_2[5].rotor_angle; //���·�:09AA   ���Ϸ�:0730   ����:2474->1840  ,������634
}

//�����ǵ�ֵ
static void gimbal_read_imu()
{
	Pitch_imu = ins_data.angle[1];   //������Pitchֵ
	Pitch_imu_speed = ins_data.gyro[1];   //�����ǽ��ٶ�ֵ
}


//ģʽѡ��
static void gimbal_choice()
{

}


//Mode_1�㷨����򵥵���̨���ƣ��ٶȻ���
static void gimbal_mode_1()
{
		if( (rc_ctrl.rc.ch[1]>1074&&rc_ctrl.rc.ch[1]<=1684 ) || (ctrl_flag))
		{
			target_speed_can_2[4]=15;
			target_speed_can_2[5]=-15;
		}
		else if( (rc_ctrl.rc.ch[1]>=324&&rc_ctrl.rc.ch[1]<974) || (shift_flag))
		{
			target_speed_can_2[4]=-15;
			target_speed_can_2[5]=15;
		}
		else
		{
			target_speed_can_2[4]=0;
			target_speed_can_2[5]=0;
		}
}	

//PID����ͷ���
static void gimbal_can_send()
{
		
  
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x2ff;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (motor_info_can_2[4].set_voltage>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[1] = (motor_info_can_2[4].set_voltage)&0xff;
  tx_data[2] = (motor_info_can_2[5].set_voltage>>8)&0xff;
  tx_data[3] = (motor_info_can_2[5].set_voltage)&0xff;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}


//��������λ����ԣ�,ֹͣ��λ
static void gimbal_limit()
{
	gimbal_read_imu();
	if( gimbal_rotor_angle >= Pitch_up)
	{
		
		target_speed_can_2[4]=-20;
		target_speed_can_2[5]=20;
	}
	
	else if( gimbal_rotor_angle <= Pitch_down)
	{
		target_speed_can_2[4]=20;
		target_speed_can_2[5]=-20;
	}
}

static void gimbal_imu_limit()
{
	if(init_pitch < -25)
	{
		init_pitch = -25;
	}
	else if(init_pitch >= -5 && Pitch_imu)
	{
		init_pitch = -5;
	}
}



//������
static void gimbal_mouse()
{
	if(mouse_y > mouse_y_valve || mouse_y < -mouse_y_valve)
	{
		target_speed_can_2[4] += (fp32)mouse_y * mouse_y_weight;
		target_speed_can_2[5] -= (fp32)mouse_y * mouse_y_weight;
	}
}

//����
static void gimbal_minipc_control()
{
	if((fp32)(Pitch_minipc) > Pitch_minipc_valve || (fp32)(Pitch_minipc) < -Pitch_minipc_valve)
	{
		target_speed_can_2[4] -= ((fp32)Pitch_minipc_fp) * Pitch_minipc_weight;
		target_speed_can_2[5] += ((fp32)Pitch_minipc_fp) * Pitch_minipc_weight;
	}
}

static void gimbal_mode_2()
{
	switch(TIM1_Mode)
	{
		case 1: target_speed_can_2[4]=-4 , target_speed_can_2[5]=4;break;
		case 2: target_speed_can_2[4]=4 , target_speed_can_2[5]=-4;break;
	}
}

static void gimbal_zero()
{
	target_speed_can_2[4] = 0;
	target_speed_can_2[5] = 0;
}

static void gimbal_mode_3()
{
		if( (rc_ctrl.rc.ch[1]>=324 && rc_ctrl.rc.ch[1] <=1684 ) )
		{
			init_pitch -= (rc_ctrl.rc.ch[1]- 1024)/660.0 * Pitch_sita_weight; 			
		}
}

static void gimbal_minipc_sita_control()
{
		init_pitch -= ((fp32)Pitch_minipc_fp) * Pitch_sita_minipc_weight;
}

static void gimbal_mode_4()
{
	switch(TIM1_Mode)
	{
		case 1: init_pitch -= 0.03f;break;
		case 2: init_pitch += 0.03f;break;
	}
}

static void gimbal_mode_5()
{
		if( (rc_ctrl.rc.ch[1]>=324 && rc_ctrl.rc.ch[1] <=1684 ) )
		{
			init_pitch -= (rc_ctrl.rc.ch[1]- 1024)/660.0 * Pitch_sita_weight * 24.0f; 			
		}
}