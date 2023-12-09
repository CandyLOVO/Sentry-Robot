#include "Yaw_task.h"
//================================================YAW轴电机控制任务================================================//

//	此文件是三云台控制任务，基于一级云台上的C板坐标系来解算二级云台的坐标
//	两个头用的CAN2,ID是1和2，左边ID是0，右边ID是1

//================================================全局变量================================================//
float target_yaw_left;	//左右脑袋的目标yaw
float target_yaw_right;
int16_t Init_encoder_left = 1000;		//左脑袋编码器正前方初始值(安装好后值固定)
int16_t Init_encoder_right = 2000;		//右脑袋
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

//================================================YAW轴控制主函数================================================//
void Yaw_task(void const *pvParameters)
{
  //参数初始化设置
	Yaw_init();
	osDelay(2000);//上电等待IMU启动成功
	
	//循环任务运行
  for(;;)
  {
		Yaw_read_imu();//获取Imu角度

    osDelay(1);
  }

}

//================================================YAW轴PID参数初始化================================================//
static void Yaw_init()
{
	pid_init(&motor_pid_can_2[0],60,0.001,0,30000,30000);
	pid_init(&motor_pid_sita_can_2[0],5,0,3,30000,30000);
	pid_init(&motor_pid_can_2[1],60,0.001,0,30000,30000);
	pid_init(&motor_pid_sita_can_2[1],5,0,3,30000,30000);
}

//================================================YAW轴角度读取===============================================//
static void Yaw_read_imu()
{
	//INS_angle[0]是C板陀螺仪的值
	//原始数据顺时针为正
	//    0
	//-180 180
	//以上为处理前
	
	//处理后逆时针为正(相对坐标系)
	int16_t Yaw_left_int = motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle);
	int16_t Yaw_right_int = motor_value(Init_encoder_left,motor_info_can_2[1].rotor_angle);
	Yaw_left = -(float)Yaw_left_int;
	Yaw_right = -(float)Yaw_right_int;
	
	//以C板上电那一刻的坐标系为基坐标系(绝对坐标系)
	Yaw_left_c = Yaw_left + INS_angle[0];
	Yaw_right_c = Yaw_right + INS_angle[0];
	//越界处理还没写！！
}

//================================================位置控制模式================================================//
static void Yaw_mode_remote_site()
{
//		if(rc_ctrl.rc.ch[0] >= 324 && rc_ctrl.rc.ch[0]<= 1684)
//		{			
//			target_yaw -= (rc_ctrl.rc.ch[0]-base)/660.0 * Yaw_sita_weight; 			
//		}
}