#include "Pitch_task.h"

//===============================================全局变量================================================//
int16_t Init_encoder_left_gimbal = 6818;		//左脑袋编码器水平时初始值(安装好后值固定)
int16_t Init_encoder_right_gimbal = 7154;		//右脑袋
float target_gimbal_left;	//左右脑袋的目标yaw（相对坐标）
float target_gimbal_right;
float Gimbal_left;
float Gimbal_right;	

//================================================函数================================================//

//初始化PID参数
static void Gimbal_init();	

//数据清零
static void Gimbal_zero();

//解算当前编码器的值
static void Gimbal_read_motor();

//遥控器控制模式(位置环)
static void Gimbal_mode_control_sita();

//巡航模式(位置环)
//static void gimbal_mode_search_sita();

//PID发送至电机
static void Gimbal_can_send();

//电流值计算
static void Gimbal_voltage_calc();

//目标值限制
static void Gimbal_target_restrict();

//实际值限位
static void Gimbal_imu_restrict();

//叠加自瞄(位置环)
static void Gimbal_minipc_control_sita();

// pitch

void Pitch_task(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	Gimbal_init();	//PID参数初始化
	osDelay(2000);
  for(;;)
  {
		Gimbal_zero();	//速度清零
		Gimbal_read_motor();	//读取编码器值
		Gimbal_minipc_control_sita();	//位置环视觉瞄准
		Gimbal_mode_control_sita();	//遥控器位置环控制模式
		Gimbal_target_restrict();	//目标值限制
//		Gimbal_imu_restrict();	//实际值限位
		Gimbal_voltage_calc();	//电流值计算
		Gimbal_can_send();
//		else if(rc_ctrl.rc.s[1]==2)		//上场模式
//		{
//			if(Sentry.foe_flag)	//如果检测到目标
//			{
//				gimbal_minipc_control_sita(); //视觉瞄准
//			}			
//			else
//			{
//				gimbal_mode_search_sita();	//哨兵巡航模式
//			}
//		}
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

//================================================初始化PID参数================================================//
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

//================================================解算当前编码器值================================================//
//注：和旧哨兵对其颗粒度，需要仰头为负数，镜像则2个6020正方向为一正一反
static void Gimbal_read_motor()
{
	Gimbal_left = -motor_value(Init_encoder_left_gimbal,motor_info_can_2[2].rotor_angle);
	Gimbal_right = motor_value(Init_encoder_right_gimbal,motor_info_can_2[3].rotor_angle);
}

//================================================Pitch数据发送================================================//
static void Gimbal_can_send()
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x1ff;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = (motor_info_can_2[0].set_voltage>>8)&0xff;	//先发高八位		
  tx_data[1] = (motor_info_can_2[0].set_voltage)&0xff;
  tx_data[2] = (motor_info_can_2[1].set_voltage>>8)&0xff;	
  tx_data[3] = (motor_info_can_2[1].set_voltage)&0xff;
  tx_data[4] = (motor_info_can_2[2].set_voltage>>8)&0xff;
  tx_data[5] = (motor_info_can_2[2].set_voltage)&0xff;
  tx_data[6] = (motor_info_can_2[3].set_voltage>>8)&0xff;
  tx_data[7] = (motor_info_can_2[3].set_voltage)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);
}

//================================================目标值限制================================================//
//仰角限位40度，俯角限位25度
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

//================================================实际值限位================================================//
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

//================================================电流值计算================================================//
static void Gimbal_voltage_calc()
{
		target_speed_can_2[2] += pid_calc_sita_span(&motor_pid_sita_can_2[2], target_gimbal_left, Gimbal_left);
		target_speed_can_2[3] -= pid_calc_sita_span(&motor_pid_sita_can_2[3], target_gimbal_right, Gimbal_right);
		motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed);
		motor_info_can_2[3].set_voltage = pid_calc(&motor_pid_can_2[3], target_speed_can_2[3], motor_info_can_2[3].rotor_speed);	
}

//================================================速度清零================================================//
static void Gimbal_zero()
{
	target_speed_can_2[2] = 0;
	target_speed_can_2[3] = 0;
}

//================================================遥控器位置环控制模式(基于陀螺仪权重)================================================//
static void Gimbal_mode_control_sita()
{
		if(rc_ctrl.rc.ch[3] >= -660 && rc_ctrl.rc.ch[3]<= 660)
		{
			target_gimbal_left -= (rc_ctrl.rc.ch[3])/660.0 * Pitch_sita_weight; 
			target_gimbal_right = target_gimbal_left;		
		}
}

//================================================视觉瞄准(位置环模式)================================================//
static void Gimbal_minipc_control_sita()
{
//	target_pitch =chase.pitch;
}

//================================================巡航模式(位置环模式)================================================//
//static void gimbal_mode_search_sita()
//{
//	switch(TIM1_Mode)
//	{
//		case 1: target_pitch -= 0.03f;break;
//		case 2: target_pitch += 0.03f;break;
//	}
//}