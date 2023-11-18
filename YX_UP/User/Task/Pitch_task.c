#include "Pitch_task.h"

//================================================Pitch轴电机控制任务================================================//

//第一版：
//此任务用来对云台进行模式选择，控制，校准等
//封装一些函数用来进行控制调用
//Pitch采用上C板CAN_2，电机ID为5
//存在问题：需要将云台Pitch锁在最下方再启动，Pitch受高频噪点影响，收敛速度太慢

//第二版：
//采用双6020结构，新加的Pitch走上C板CAN_2，电机ID为6
//Pitch值是负的，抬头绝对值增大(但是是负数)


//===============================================全局变量================================================//

uint16_t gimbal_rotor_angle;		//ID为5的6020编码器值
uint16_t gimbal_rotor_angle_2;		//ID为6的6020编码器值
float Pitch_imu;		//pitch imu解算数据
float Pitch_imu_speed;		//pitch imu角速度 
float target_pitch;		//目标pitch
float ins_pitch_speed;

//================================================函数================================================//

//初始化PID参数
static void gimbal_init();	

//数据清零
static void gimbal_zero();

//读取imu参数
static void gimbal_read_imu();

//读取编码器的值
static void gimbal_read_motor();

//遥控器控制模式(速度环)
static void gimbal_control_speed();

//巡航模式(速度环)
static void gimbal_mode_search_speed();

//遥控器控制模式(位置环)
static void gimbal_mode_control_sita();

//巡航模式(位置环)
static void gimbal_mode_search_sita();

//PID发送至电机
static void gimbal_can_send();

//限位（编码器）（未使用）
static void gimbal_limit();

//限位（陀螺仪）
static void gimbal_imu_limit();

//鼠标控制Pitch(叠加)
static void gimbal_mouse();

//叠加自瞄(速度环)
static void gimbal_minipc_control();

//叠加自瞄(位置环)
static void gimbal_minipc_control_sita();

// pitch

void Pitch_task(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */

	gimbal_init();	//PID参数初始化
	osDelay(3000);
  for(;;)
  {
		gimbal_zero();	//速度清零
		gimbal_read_motor();	//读取编码器值
		gimbal_read_imu();	//读取Imu值
		if(rc_ctrl.rc.s[1]==3 || rc_ctrl.rc.s[1]==1	)	//调试模式
		{
			gimbal_minipc_control_sita();	//位置环视觉瞄准
			gimbal_mode_control_sita();	//遥控器位置环控制模式
		 }
		  
		else if(rc_ctrl.rc.s[1]==2)		//上场模式
		{
			if(Sentry.foe_flag)	//如果检测到目标
			{
				gimbal_minipc_control_sita(); //视觉瞄准
			}			
			else
			{
				gimbal_mode_search_sita();	//哨兵巡航模式
			}
		}
		gimbal_imu_limit();	//软件限位
		target_speed_can_2[4] -= pid_calc_sita(&motor_pid_sita_can_2[4], target_pitch, Pitch_imu);	//imu位置环解算
		//target_speed_can_2[5] += pid_calc_sita(&motor_pid_sita_can_2[5], target_pitch, Pitch_imu);
		
//		编码器解算模式
//		target_speed_can_2[4] -= pid_calc_sita(&motor_pid_sita_can_2[4], target_pitch*(-22.75)+3632, motor_info_can_2[4].rotor_angle);	//编码器位置环解算(使用这个时注意遥控器映射权重)
//		target_speed_can_2[5] += pid_calc_sita(&motor_pid_sita_can_2[5], target_pitch*(22.75)+2474, motor_info_can_2[5].rotor_angle);


		ins_pitch_speed = Pitch_imu_speed * 9.55f;	//统一速度环单位：以编码器为基准，rad/s -> round/min	60/(2*pi)=9.55
		motor_info_can_2[4].set_voltage = pid_calc(&motor_pid_can_2[4], target_speed_can_2[4], ins_pitch_speed);
		motor_info_can_2[5].set_voltage = -motor_info_can_2[4].set_voltage;
		//motor_info_can_2[5].set_voltage = pid_calc(&motor_pid_can_2[5], target_speed_can_2[5], ins_pitch_speed );
		
//		编码器解算模式
//		motor_info_can_2[4].set_voltage = pid_calc(&motor_pid_can_2[4], target_speed_can_2[4], motor_info_can_2[4].rotor_speed);
//		motor_info_can_2[5].set_voltage = pid_calc(&motor_pid_can_2[5], target_speed_can_2[5], motor_info_can_2[5].rotor_speed);
			gimbal_can_send();
		
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

//================================================初始化PID参数================================================//
static void gimbal_init()	
{
	
	pid_init(&motor_pid_can_2[4],120,0.001,5,30000,30000);// 120 0.01 0
	pid_init(&motor_pid_can_2[5],120,0.001,5,30000,30000);// 120 0.01 0
	pid_init(&motor_pid_sita_can_2[4],18,0,350,30000,30000);// 10 0 1300
	pid_init(&motor_pid_sita_can_2[5],18,0,350,30000,30000);// 10 0 1300
	target_pitch = Pitch_imu;
} 

//================================================读取编码器值================================================//
static void gimbal_read_motor()
{
	gimbal_rotor_angle = motor_info_can_2[4].rotor_angle;	//最下方:0E30   最上方:10B0  换算:3632->4272  ,步长：640
	gimbal_rotor_angle_2 = motor_info_can_2[5].rotor_angle; //最下方:09AA   最上方:0730   换算:2474->1840  ,步长：634
}

//================================================读取imu值================================================//
static void gimbal_read_imu()
{
	Pitch_imu = INS_angle[1];   //陀螺仪Pitch值
	Pitch_imu_speed = INS_gyro[1];   //陀螺仪角速度值
}


//================================================速度环控制================================================//
static void gimbal_control_speed()
{
		if( (rc_ctrl.rc.ch[1]>1074&&rc_ctrl.rc.ch[1]<=1684 ) || (remote.key.ctrl))
		{
			target_speed_can_2[4]=15;
			target_speed_can_2[5]=-15;
		}
		else if( (rc_ctrl.rc.ch[1]>=324&&rc_ctrl.rc.ch[1]<974) || (remote.key.shift))
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

//================================================Pitch数据发送================================================//
static void gimbal_can_send()
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x2ff;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = (motor_info_can_2[4].set_voltage>>8)&0xff;	//先发高八位		
  tx_data[1] = (motor_info_can_2[4].set_voltage)&0xff;
  tx_data[2] = (motor_info_can_2[5].set_voltage>>8)&0xff;
  tx_data[3] = (motor_info_can_2[5].set_voltage)&0xff;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}


//================================================基于编码器的反转限位(未使用)================================================//
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

//================================================Pitch软件限位================================================//
static void gimbal_imu_limit()
{
	if(target_pitch < Up_inf && Pitch_imu)
	{
		target_pitch = Up_inf;
	}
	else if(target_pitch >= Down_inf && Pitch_imu)
	{
		target_pitch = Down_inf;
	}
}

//================================================鼠标控制(未使用)================================================//
static void gimbal_mouse()
{
	if(remote.mouse.y > mouse_y_valve || remote.mouse.y < -mouse_y_valve)
	{
		target_speed_can_2[4] += (fp32)remote.mouse.y * mouse_y_weight;
		target_speed_can_2[5] -= (fp32)remote.mouse.y * mouse_y_weight;
	}
}

//================================================视觉瞄准(速度环模式)================================================//
static void gimbal_minipc_control()
{
		target_speed_can_2[4] -= chase.pitch * Pitch_minipc_weight;
		target_speed_can_2[5] += chase.pitch * Pitch_minipc_weight;
}

//================================================巡航模式(速度环模式)================================================//
static void gimbal_mode_search_speed()
{
	switch(TIM1_Mode)
	{
		case 1: target_speed_can_2[4]=-4 , target_speed_can_2[5]=4;break;
		case 2: target_speed_can_2[4]=4 , target_speed_can_2[5]=-4;break;
	}
}

//================================================速度清零================================================//
static void gimbal_zero()
{
	target_speed_can_2[4] = 0;
	target_speed_can_2[5] = 0;
}

//================================================遥控器位置环控制模式(基于陀螺仪权重)================================================//
static void gimbal_mode_control_sita()
{
		if( (rc_ctrl.rc.ch[1]>=324 && rc_ctrl.rc.ch[1] <=1684 ) )
		{
			target_pitch -= (rc_ctrl.rc.ch[1]- 1024)/660.0 * Pitch_sita_weight; 			
		}
}

//================================================视觉瞄准(位置环模式)================================================//
static void gimbal_minipc_control_sita()
{
	//target_pitch -= (chase.pitch - target_pitch) * Pitch_sita_minipc_weight;
	target_pitch =chase.pitch;
}

//================================================巡航模式(位置环模式)================================================//
static void gimbal_mode_search_sita()
{
	switch(TIM1_Mode)
	{
		case 1: target_pitch -= 0.03f;break;
		case 2: target_pitch += 0.03f;break;
	}
}