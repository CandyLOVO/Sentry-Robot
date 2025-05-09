#include "Yaw_task.h"
#include "Ins_task.h"
//================================================YAW轴电机控制任务================================================//

//	此文件是三云台控制任务，基于一级云台上的C板坐标系来解算二级云台的坐标
//	两个头用的CAN2,ID是1和2，左边ID是0，右边ID是1
//	绝对基坐标系是底盘C板上电的坐标系

//================================================全局变量================================================//
float target_yaw_left;	//左右脑袋的目标yaw（相对坐标）
float target_yaw_right;
float target_yaw_remote_left; //遥控器提供的目标值
float target_yaw_remote_right;
float target_yaw_middle ; //9025电机转动的目标值
int m = 0;

//需要修改对应的数值，根据安装后读取的电机编码值修改
int16_t Init_encoder_left = 7904;		//左脑袋编码器正前方初始值(安装好后值固定)
int16_t Init_encoder_right = 3640;		//右脑袋
uint16_t Init_encoder_middle = 5072; //一级云台,正前方要和底盘C板正前方朝向一致

float Yaw_middle_c;	//一级云台yaw(只有绝对坐标) 9025转化为0~+-180后的编码值
//逆时针：0~180,-180~0
float Yaw_value = 0;
float Yaw_left;	//现在时刻左脑袋的yaw（相对坐标） 编码值转化为0~+-180后的编码值
float Yaw_right;	//编码值转化为0~+-180后的编码值
float Yaw_left_c;	//现在时刻左脑袋的yaw（绝对坐标） 相对于整车IMU正方向的角度值
float Yaw_right_c; //相对于整车正方向的角度值

int8_t get_flag = 0; //两个头都没识别到0，左头识别到1，右头识别到2
//================================================函数================================================//

//初始化PID参数
static void Yaw_init();	

//每次循环初始化
static void Yaw_loop_init();

//获取C板IMU值
static void Yaw_read_imu();

//两个脑袋的位置控制模式
static void Yaw_mode_remote_site();

//两个脑袋的巡航模式
static void Yaw_mode_searching();

//遥控器相对角度限制
static void Yaw_remote_restrict();

//死区角度限制
static void Yaw_target_restrict();

//速度环计算
static void Yaw_speed_calc();

//电压环计算
static void Yaw_voltage_calc();

//循环初始化
static void Yaw_loop_init();

//MF9025位置控制
static void Site_Control_MF();

//模式选择
static void Yaw_mode_judge();

//9025电流环计算
static void Voltage_Control_MF();

//9025巡航模式
static void Searching_Control_MF();

//Mode2自瞄响应时计算偏差角
float Delta_calc(float distance);

//================================================YAW轴控制主函数================================================//
void Yaw_task(void const *pvParameters)
{
  //参数初始化设置
	osDelay(2000);//上电等待IMU启动成功
	motor_info_can_2[7].can_id = 1;//初始化9025电机ID
	Start_MF_send(motor_info_can_2[7].can_id);//初始化9025 启动电机
	
	Yaw_init(); //PID参数初始化
	
	osDelay(10);
	
	//循环任务运行
  for(;;)
  {
		Yaw_loop_init();//循环初始化三个电机的速度环输入为0
		
		//三个电机编码值转化
		Yaw_read_imu();//获取Imu角度
		
		//模式选择，计算目标值
		Yaw_mode_judge();//模式选择
		Yaw_target_restrict();//目标角度限制(目标角度进入死区时，自瞄和上电初始化时专用)
		
		//PID
		Yaw_speed_calc();//角度环计算（带有相对角度限制处理）->速度环输入值
		Yaw_voltage_calc();//电压环计算（速度环）
		
		//CAN发送
		Yaw_can_send();//发送6020
		Voltage_Control_MF();//电流环计算
		Current_Control_MF_send(motor_info_can_2[7].can_id,motor_info_can_2[7].set_voltage);//发送9025
    osDelay(1);
  }

}

//================================================YAW轴PID参数和目标IMU初始化================================================//
static void Yaw_init()
{
	pid_init(&motor_pid_can_2[7],800,0.01,0,30000,30000); //9025电机速度环
	pid_init(&motor_pid_sita_can_2[7],5.5,0,20,30000,30000); //9025电机角度环
	
//上场用PID 硬的一批
	pid_init(&motor_pid_can_2[0],200,0.01,0,30000,30000); //左头速度环
	pid_init(&motor_pid_sita_can_2[0],25,0,10,30000,30000); //左头角度环
	
	pid_init(&motor_pid_can_2[1],200,0.01,0,30000,30000); //右头速度环
	pid_init(&motor_pid_sita_can_2[1],25,0,10,30000,30000); //右头角度环
	
//调试用PID
//	pid_init(&motor_pid_can_2[0],94,0,0,30000,30000); //左头速度环
//	pid_init(&motor_pid_sita_can_2[0],1,0,0,30000,30000); //左头角度环
//	
//	pid_init(&motor_pid_can_2[1],1,0,0,30000,30000); //右头速度环
//	pid_init(&motor_pid_sita_can_2[1],1,0,0,30000,30000); //右头角度环
	
	Encoder_MF_read(motor_info_can_2[7].can_id);//9025读取当前编码器值
	Yaw_value = MF_value(Init_encoder_middle , motor_info_can_2[7].rotor_angle , 65535); //将9025编码值转换到-180~0、0~180
	
	Yaw_left = -motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle); //将6020编码值转换到-180~0、0~180
	Yaw_right = -motor_value(Init_encoder_right,motor_info_can_2[1].rotor_angle);
	target_yaw_middle = Yaw_middle_c;
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
	
	//三个电机编码值转化到0~+-180
	Yaw_value = MF_value(Init_encoder_middle , motor_info_can_2[7].rotor_angle , 65535); //将9025编码值转换到-180~0、0~180
	Yaw_left = -motor_value(Init_encoder_left,motor_info_can_2[0].rotor_angle);
	Yaw_right = -motor_value(Init_encoder_right,motor_info_can_2[1].rotor_angle);
	
	//以C板上电那一刻的坐标系为基坐标系(绝对坐标系)
	Yaw_left_c = Yaw_left + Yaw_middle_c;
	Yaw_right_c = Yaw_right + Yaw_middle_c;
	//越界处理
	if(Yaw_left_c>180)
		Yaw_left_c-=360;
	else if(Yaw_left_c<-180)
		Yaw_left_c+=360;
	
	if(Yaw_right_c>180)
		Yaw_right_c-=360;
	else if(Yaw_right_c<-180)
		Yaw_right_c+=360;	
}

//================================================位置控制模式================================================//
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

//================================================巡航模式================================================//
static void Yaw_mode_searching()
{
	if(Sentry.L_Flag_yaw_direction == 1)
	{
		target_yaw_remote_left+=0.09;
		if(target_yaw_remote_left>=10)
		{
			Sentry.L_Flag_yaw_direction=2;
			target_yaw_remote_left-=0.09;
		}
	}
	else if(Sentry.L_Flag_yaw_direction == 2)
	{
		target_yaw_remote_left-=0.09;
		if(target_yaw_remote_left<=-190)
		{
			Sentry.L_Flag_yaw_direction=1;
			target_yaw_remote_left+=0.09;
		}		
	}
	
	if(Sentry.R_Flag_yaw_direction == 1)
	{
		target_yaw_remote_right-=0.09;
		if(target_yaw_remote_right<=-10)
		{
			Sentry.R_Flag_yaw_direction=2; //反方向旋转
			target_yaw_remote_right+=0.09;
		}
	}
	else if(Sentry.R_Flag_yaw_direction == 2)
	{
		target_yaw_remote_right+=0.09;
		if(target_yaw_remote_right>=190)
		{
			Sentry.R_Flag_yaw_direction=1;
			target_yaw_remote_right-=0.09;
		}		
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
  tx_data[4] = (motor_info_can_2[2].set_voltage>>8)&0xff;
  tx_data[5] = (motor_info_can_2[2].set_voltage)&0xff;
  tx_data[6] = (motor_info_can_2[3].set_voltage>>8)&0xff;
  tx_data[7] = (motor_info_can_2[3].set_voltage)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);
}

//================================================控制角度限制================================================//
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

//================================================6020目标角度限制===============================================//
static void Yaw_target_restrict()
{
	//限制目标角度
	if(target_yaw_left>10 && target_yaw_left<170)
	{
		target_yaw_left=0;
	}
	if(target_yaw_right<-10 && target_yaw_right>-170)
	{
		target_yaw_right=0;
	}
}


//================================================速度环输入计算（倒装6020取负值）================================================//
static void Yaw_speed_calc()
{
	target_speed_can_2[0] +=  pid_calc_sita_span_left(&motor_pid_sita_can_2[0], target_yaw_left, Yaw_left);
	target_speed_can_2[1] +=  pid_calc_sita_span_right(&motor_pid_sita_can_2[1], target_yaw_right, Yaw_right);
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
	target_speed_can_2[7] = 0;
	target_speed_can_2[0] = 0;
	target_speed_can_2[1] = 0;
}
	
//================================================MF9025位置控制===============================================//
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

//================================================MF9025电流环计算(调用了限制函数)===============================================//
static void Voltage_Control_MF()
{
	target_speed_can_2[7] += pid_calc_sita_span(&motor_pid_sita_can_2[7], target_yaw_middle, Yaw_middle_c);  //Yaw_middle_c->IMU -180~+180
//	motor_info_can_2[7].set_voltage = pid_calc(&motor_pid_can_2[7], target_speed_can_2[7],GZ);
	motor_info_can_2[7].set_voltage = pid_calc(&motor_pid_can_2[7], target_speed_can_2[7],motor_info_can_2[7].rotor_speed);
}

//================================================9025巡航模式===============================================//
static void Searching_Control_MF()
{
	target_yaw_middle+=0.03;
	if(target_yaw_middle>180)
	{
		target_yaw_middle-=360;
	}
}

//================================================Yaw控制模式旋转===============================================//
static void Yaw_mode_judge()
{
	if(Sentry.Remote_mode==33)
	{
		Site_Control_MF();//MF9025位置模式(遥控器)
		target_yaw_remote_left = 0;
		target_yaw_remote_right = 0;
		target_yaw_left = target_yaw_remote_left;
		target_yaw_right = target_yaw_remote_right;
		Yaw_remote_restrict(); //遥控器数据限制[-20,200] 并将处理后的数值赋给电机目标值
	}
	else if(Sentry.Remote_mode==13) //左杆控制左头，右杆控制右头，小yaw相对位置不变
	{
		target_yaw_middle=0;
		Yaw_mode_remote_site();//位置控制模式
		Yaw_remote_restrict();//遥控器控制下的目标角度限制 并将处理后的数值赋给电机目标值
	}
	
	
	else if(Sentry.Remote_mode==22)	//上场模式
	{
		//两个头都没有识别到
		if(vision_receive.L_tracking == 0 && vision_receive.R_tracking == 0)
		{
			get_flag = 0;
			Searching_Control_MF(); //9025巡航0.05
			Yaw_mode_searching(); //执行一次小yaw正/反转0.09度
			target_yaw_left = target_yaw_remote_left;
			target_yaw_right = target_yaw_remote_right;
		}
		
		//左头识别到
		else if(vision_receive.L_tracking == 1 && vision_receive.R_tracking == 0)
		{
			get_flag = 1; //左头识别到
			target_yaw_middle = vision_receive.yaw_L; //左头视觉传来大yaw的值
			target_yaw_left = vision_receive.L_chase_yaw - Yaw_middle_c; //左头小yaw相对目标角度
			target_yaw_right = 0; //右头转到与大yaw平行
		}
		
		//右头识别到
		else if(vision_receive.L_tracking == 0 && vision_receive.R_tracking == 1)
		{
			get_flag = 2; //右头识别到
			target_yaw_middle = vision_receive.yaw_R; //右头视觉传来大yaw的值
			target_yaw_right = vision_receive.R_chase_yaw - Yaw_middle_c; //右头小yaw相对目标角度
			target_yaw_left = 0; //左头转到与大yaw平行
		}
		
		else if(vision_receive.L_tracking == 1 && vision_receive.R_tracking == 1)
		{
			if(get_flag == 0 || get_flag == 1)
			{
				target_yaw_middle = vision_receive.yaw_L; //左头视觉传来大yaw的值
				target_yaw_left = vision_receive.L_chase_yaw - Yaw_middle_c; //左头小yaw相对目标角度
				target_yaw_right = vision_receive.R_chase_yaw - Yaw_middle_c; //右头小yaw相对目标角度
			}
			else if(get_flag == 2)
			{
				target_yaw_middle = vision_receive.yaw_R; //右头视觉传来大yaw的值
				target_yaw_right = vision_receive.R_chase_yaw - Yaw_middle_c; //右头小yaw相对目标角度
				target_yaw_left = vision_receive.L_chase_yaw - Yaw_middle_c; //左头小yaw相对目标角度
			}
		}
		//越界处理
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

//================================================Mode3自瞄响应时计算偏差角(单位统一为m，角度制)===============================================//
float Delta_calc(float distance)
{
	float Delta = 0;
	float d = 0.1023885;
	Delta = (float)asin(d/distance) * 57.3f;
	Delta = fabs(Delta);
	return Delta;
}
