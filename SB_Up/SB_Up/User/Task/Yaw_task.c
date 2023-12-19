#include "Yaw_task.h"
//================================================YAW轴电机控制任务================================================//

//	此文件是三云台控制任务，基于一级云台上的C板坐标系来解算二级云台的坐标
//	两个头用的CAN2,ID是1和2，左边ID是0，右边ID是1
//	绝对基坐标系是底盘C板上电的坐标系

//================================================全局变量================================================//
float target_yaw_left;	//左右脑袋的目标yaw（相对坐标）
float target_yaw_right;
float target_yaw_middle;
int16_t Init_encoder_left = 6818;		//左脑袋编码器正前方初始值(安装好后值固定)
int16_t Init_encoder_right = 7154;		//右脑袋
int16_t Init_encoder_middle = 15693;		//一级云台,正前方要和底盘C板正前方朝向一致
float Yaw_middle;	//一级云台yaw(只有绝对坐标)
float Yaw_left;	//现在时刻左脑袋的yaw（相对坐标）
float Yaw_right;	
float Yaw_left_c;	//现在时刻左脑袋的yaw（绝对坐标）
float Yaw_right_c;	
//================================================函数================================================//

//初始化PID参数
static void Yaw_init();	

//每次循环初始化
static void Yaw_loop_init();

//获取C板IMU值
static void Yaw_read_imu();

//两个脑袋的位置控制模式
static void Yaw_mode_remote_site();

//相对角度限制
static void Yaw_restrict();

//速度环计算
static void Yaw_speed_calc();

//电压环计算
static void Yaw_voltage_calc();

//循环初始化
static void Yaw_loop_init();

//MF9025位置控制
static void Site_Control_MF();

//MF9025力控(等比缩放)
static void Current_Control_MF();
	
//================================================YAW轴控制主函数================================================//
void Yaw_task(void const *pvParameters)
{
  //参数初始化设置
	osDelay(2000);//上电等待IMU启动成功
	motor_info[0].can_id = 1;//初始化9025电机ID
	Start_MF_send(motor_info[0].can_id);//初始化9025
	Yaw_init();
	osDelay(10);
	
	//循环任务运行
  for(;;)
  {
		Yaw_loop_init();//循环初始化
		Yaw_read_imu();//获取Imu角度
		Yaw_mode_remote_site();//位置控制模式
		Encoder_MF_read(motor_info[0].can_id);//读取当前编码器值，读完之后用下面那一行的位置模式
		Site_Control_MF();//MF9025位置模式(遥控器)
//		Current_Control_MF();//MF9025力控模式(遥控器)
		Yaw_restrict();//相对角度限制
		Yaw_speed_calc();//速度环计算
		Yaw_voltage_calc();//电压环计算
		Yaw_can_send();
		Current_Control_MF_send(motor_info[0].can_id,motor_info[0].set_voltage);
    osDelay(1);
  }

}

//================================================YAW轴PID参数和目标IMU初始化================================================//
static void Yaw_init()
{
//	pid_init(&motor_pid[0],3,0,1,2048,2048);
	pid_init(&motor_pid_sita[0],6,0.01,0,2048,2048);
	
	pid_init(&motor_pid_can_2[0],30,0.001,0,30000,30000);
	pid_init(&motor_pid_sita_can_2[0],3,0,1,30000,30000);
	pid_init(&motor_pid_can_2[1],30,0.001,0,30000,30000);
	pid_init(&motor_pid_sita_can_2[1],3,0,1,30000,30000);
	
	Encoder_MF_read(motor_info[0].can_id);//读取当前编码器值
	Yaw_middle = MF_value(Init_encoder_middle , motor_info[0].rotor_angle , 65535);
	
	int16_t Yaw_left_int = motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle);
	int16_t Yaw_right_int = motor_value(Init_encoder_right,motor_info_can_2[1].rotor_angle);
	Yaw_left = (float)Yaw_left_int;
	Yaw_right = (float)Yaw_right_int;
	
	target_yaw_middle = Yaw_middle;
	target_yaw_left = Yaw_left;
	target_yaw_right = Yaw_right;

	
}

//================================================YAW轴角度读取===============================================//
static void Yaw_read_imu()
{
	//INS_angle[0]是C板陀螺仪的值
	//原始数据顺时针为正
	//处理后逆时针为正(相对坐标系)
	//    0
	//180 -180
	
	Yaw_middle = MF_value(Init_encoder_middle , motor_info[0].rotor_angle , 65535);
	
	int16_t Yaw_left_int = motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle);
	int16_t Yaw_right_int = motor_value(Init_encoder_right,motor_info_can_2[1].rotor_angle);
	Yaw_left = (float)Yaw_left_int;
	Yaw_right = (float)Yaw_right_int;
	
	//以C板上电那一刻的坐标系为基坐标系(绝对坐标系)
	Yaw_left_c = Yaw_left + Yaw_middle;
	Yaw_right_c = Yaw_right + Yaw_middle;
	//越界处理还没写！！
	
}

//================================================位置控制模式================================================//
static void Yaw_mode_remote_site()
{
		if(rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1]<= 660)
		{			
			target_yaw_left -= rc_ctrl.rc.ch[1]/660.0 * Yaw_sita_weight; 	
			target_yaw_right = -target_yaw_left;
		}
}

//================================================Yaw电机电流发送================================================//
static void Yaw_can_send()
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
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);
}

//================================================Yaw角度限制================================================//
static void Yaw_restrict()
{
	if(target_yaw_left<0)
	{
		target_yaw_left=0; 
	}
	else if(target_yaw_left>180)
	{
		target_yaw_left=180; 
	}
	
	if(target_yaw_right>0)
	{
		target_yaw_left=0; 
	}
	else if(target_yaw_left<-180)
	{
		target_yaw_left=-180; 
	}
}

//================================================速度环输入计算（倒装6020取负值）================================================//
static void Yaw_speed_calc()
{
	target_speed_can_2[0] -=  pid_calc_sita(&motor_pid_sita_can_2[0], target_yaw_left, Yaw_left);
	target_speed_can_2[1] -=  pid_calc_sita(&motor_pid_sita_can_2[1], target_yaw_right, Yaw_right);
}

//================================================电压环计算================================================//
static void Yaw_voltage_calc()
{
	motor_info_can_2[0].set_voltage = pid_calc(&motor_pid_can_2[0], target_speed_can_2[0], motor_info_can_2[0].rotor_speed);
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[1].rotor_speed);
}

//================================================循环初始化================================================//
static void Yaw_loop_init()
{
	target_speed_can_2[0] = 0;
	target_speed_can_2[1] = 0;
}
	
//================================================MF9025位置控制===============================================//
static void Site_Control_MF()
{
	if(rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0]<= 660)
	{
		target_yaw_middle -= rc_ctrl.rc.ch[0]/660.0 * Yaw_sita_weight;
		motor_info[0].set_voltage = pid_calc_sita(&motor_pid_sita[0], target_yaw_middle, Yaw_middle);
	}
	
	//限制函数调用
	motor_info[0].set_voltage = Current_Limit_MF(motor_info[0].set_voltage);
}

//================================================MF9025力控(等比缩放)===============================================//
static void Current_Control_MF()
{
	if(rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0]<= 660)
	{
		motor_info[0].set_voltage = 2048*((float)rc_ctrl.rc.ch[0]/660);
	}
	//限制函数调用
	motor_info[0].set_voltage = Current_Limit_MF(motor_info[0].set_voltage);
}