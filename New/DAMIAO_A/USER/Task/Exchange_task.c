#include "Exchange_task.h"

//================================================通信任务================================================//
//注：视觉的运动预测和弹道补偿均已转移到上位机了，下位机未使用了

//================================================函数================================================//
//向can1发送信息
static void Send_to_CAN1();

//获取键盘信息
static void Get_keyboard();

//获取Nuc的信息，已转移至中断
static void Get_minipc();

//判断上位机检测到目标，检测到就进行解算，没检测到赋0
static void Judge_minipc();

//对来自Nuc的信息进行解码
void Vision_read(uint8_t rx_buffer[]);

//向Nuc发送信息
static void Stm_pc_send();

//弹道补偿API接口初始化
static void SolveTrajectory_Init();

//哨兵状态量及官方裁判系统数据初始化
static void Sentry_Init();

//视觉发送初始化
static void Vision_Init();

//================================================全局变量================================================//
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
volatile uint8_t rx_len_uart4 = 0;  //接收一帧数据的长度
volatile uint8_t recv_end_flag_uart4 = 0; //一帧数据接收完成标志
uint8_t rx_buffer[100]={0};  //接收数据缓存数组
uint8_t vision_send_L[100];	//视觉接口发送数据帧
uint8_t vision_send_R[100];

Vision_t vision;	//视觉数据发送结构体
Vision_receive_t vision_receive;	//视觉数据接收结构体
remote_flag_t remote;	//键盘按键读取(结构体)
Sentry_t Sentry;	//哨兵状态量和裁判系统数据结构体


void Exchange_task(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	Vision_Init();
	Sentry_Init();	//哨兵状态量及裁判系统数据初始化
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE); //使能uart4的IDLE中断
	HAL_UART_Receive_DMA(&huart4,rx_buffer,100); //开启接收
  for(;;)
  {
		osDelay(1);
		Send_to_CAN1();	//向can1上发送信息
		Get_keyboard();		//解算键盘的信息
		Judge_minipc();		//通过信息更新哨兵状态结构体
		Stm_pc_send();		//向nuc发送信息
  }
  /* USER CODE END StartTask03 */
} 

//================================================获取键盘数据================================================//
//注：can接收后会先储存在rc_ctrl这个结构体里(官方结构体)，这里将它取出来统一处理了
static void Get_keyboard()	
{
		memcpy(&remote.key , &rc_ctrl.key , 2);
		remote.mouse.press_left = rc_ctrl.mouse.press_l;
		remote.mouse.press_right = rc_ctrl.mouse.press_r;
	  remote.mouse.x = rc_ctrl.mouse.x;
		remote.mouse.y = rc_ctrl.mouse.y;
}

//================================================通信读取解算任务================================================//
void Vision_read(uint8_t rx_buffer[])
{
	memcpy(&vision_receive.L_tracking,&rx_buffer[1],1);
	memcpy(&vision_receive.L_shoot,&rx_buffer[2],1);
	memcpy(&vision_receive.L_chase_yaw,&rx_buffer[3],4);
	memcpy(&vision_receive.L_chase_pitch,&rx_buffer[7],4);
	memcpy(&vision_receive.L_distance,&rx_buffer[11],4);
	memcpy(&vision_receive.naving,&rx_buffer[17],1);
	memcpy(&vision_receive.nav_vx,&rx_buffer[18],4);
	memcpy(&vision_receive.nav_vy,&rx_buffer[22],4);
}

//================================================数据stm32 -> 上位机================================================//
static void Stm_pc_send()
{
	//更新姿态数据
	vision.L_pitch = Gimbal_left;
	vision.L_yaw = Yaw_left_c; //现在时刻左脑袋的yaw（绝对坐标） 相对于整车IMU正方向的角度值
	vision.R_pitch = Gimbal_right;
	vision.R_yaw = Yaw_right_c;
	
	memcpy(&vision_send_L[0],&vision.header,1);
	memcpy(&vision_send_L[1],&Sentry.Flag_judge,1); //红蓝方检测，置0为裁判系统寄了，置1为我方是红色方，置2为我方是蓝色方
	memcpy(&vision_send_L[2],&vision.L_yaw,4);
	memcpy(&vision_send_L[6],&vision.L_pitch,4);
	vision.checksum_L = Get_CRC16_Check_Sum(vision_send_L,10,0xffff);
	memcpy(&vision_send_L[10],&vision.checksum_L,2);
//	memcpy(&vision_send_L[17],&Sentry.Flag_mode,1);//哨兵目前的模式
//	memcpy(&vision_send_L[18],&Sentry.Flag_progress,1);//裁判系统比赛进程数据
	memcpy(&vision_send_L[12],&vision.ending,1);
//	HAL_UART_Transmit_DMA(&huart4,vision_send_L,13);
	HAL_UART_Transmit(&huart4,vision_send_L,13,0xff);
	
	memcpy(&vision_send_R[0],&vision.header,1);
	memcpy(&vision_send_R[1],&Sentry.Flag_judge,1); //红蓝方检测，置0为裁判系统寄了，置1为我方是红色方，置2为我方是蓝色方
	memcpy(&vision_send_R[2],&vision.R_yaw,4);
	memcpy(&vision_send_R[6],&vision.R_pitch,4);
	//crc
	vision.checksum_R = Get_CRC16_Check_Sum(vision_send_R,10,0xffff);
	memcpy(&vision_send_R[10],&vision.checksum_R,2);
	memcpy(&vision_send_R[12],&vision.ending,1);
//	HAL_UART_Transmit_DMA(&huart5,vision_send_R,13);
	HAL_UART_Transmit(&huart5,vision_send_R,13,0xff);
}

//================================================弹道补偿API接口================================================//
//static void SolveTrajectory_Init()
//{
//		//    //定义参数
//    st.k = 0.092;
//    st.bullet_type =  BULLET_17;
//    st.current_v = 18;
//    st.current_pitch = 0;
//    st.current_yaw = 0;
//    st.xw = 3.0;
//    // st.yw = 0.0159;
//    st.yw = 0;
//    // st.zw = -0.2898;
//    st.zw = 1.5;

//    st.vxw = 0;
//    st.vyw = 0;
//    st.vzw = 0;
//    st.v_yaw = 0;
//    st.tar_yaw = 0.09131;
//    st.r1 = 0.5;
//    st.r2 = 0.5;
//    st.dz = 0.1;
//    st.bias_time = 100;
//    st.s_bias = 0.00;
//    st.z_bias = -0.08;
//    st.armor_id = ARMOR_INFANTRY3;
//    st.armor_num = 2;//ARMOR_NUM_NORMAL;
//		
//		//初始化vision
//		vision.header = 0x5A;
//		vision.official.detect_color = 1;	//读取裁判系统数据判断红蓝方
//		vision.official.reset_tracker = 0;
//		vision.official.reserved = 6;
//		vision.roll = INS_angle[2]/57.3f;
//		vision.pitch = INS_angle[1]/57.3f;
//		vision.yaw = INS_angle[0]/57.3f;
//		vision.aim_x = 0.5;
//		vision.aim_y = 0.5;
//		vision.aim_z = 5;
//		vision.checksum = 0xAAAA;	//CRC16校验，我没用，发了个定值做校验
//		
//}

//================================================向can1上发送信息================================================//
static void Send_to_CAN1()
{
		uint8_t ins_buf[8] = {0};
		//0x51，陀螺仪值(范围为-180到180)和导航标志位
		memcpy(&ins_buf[0],&Yaw_middle_c,4);
		memcpy(&ins_buf[4],&vision_receive.naving,1);
		can_remote(ins_buf,0x51);
		osDelay(1);
		
		//0x52，导航的x和y值
		memcpy(&ins_buf[0],&vision_receive.nav_vx,4);
		memcpy(&ins_buf[4],&vision_receive.nav_vy,4);
		can_remote(ins_buf,0x52);
		osDelay(1);
		
		//0x53，哨兵状态信息
		memcpy(&ins_buf[0],&Sentry.Flag_mode,1);//目前的自瞄模式
		memcpy(&ins_buf[1],&Sentry.L_Flag_foe,1);//左头视觉识别标志位
		memcpy(&ins_buf[2],&Sentry.R_Flag_foe,1);//右头视觉识别标志位
		memcpy(&ins_buf[3],&Yaw_value,4);//9025编码值（转化为0~+-180）
		can_remote(ins_buf,0x53);
		osDelay(1);
}

//================================================通过上位机数据更新哨兵状态结构体================================================//
static void Judge_minipc()
{
	//遥控器模式判断
	if(rc_ctrl.rc.s[1]==1 && rc_ctrl.rc.s[0]==1)
		Sentry.Remote_mode = 11;
	else if(rc_ctrl.rc.s[1]==1 && rc_ctrl.rc.s[0]==3)
		Sentry.Remote_mode = 13;
	else if(rc_ctrl.rc.s[1]==1 && rc_ctrl.rc.s[0]==2)
		Sentry.Remote_mode = 12;
	
	else if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==1)
		Sentry.Remote_mode = 31;
	else if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==3)
		Sentry.Remote_mode = 33;
	else if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==2)
		Sentry.Remote_mode = 32;
	
	else if(rc_ctrl.rc.s[1]==2 && rc_ctrl.rc.s[0]==1)
		Sentry.Remote_mode = 21;
	else if(rc_ctrl.rc.s[1]==2 && rc_ctrl.rc.s[0]==3)
		Sentry.Remote_mode = 23;
	else if(rc_ctrl.rc.s[1]==2 && rc_ctrl.rc.s[0]==2)
		Sentry.Remote_mode = 22;
	
	else if(rc_ctrl.rc.s[1]==0 && rc_ctrl.rc.s[0]==0)//未开启遥控器默认为上场模式
		Sentry.Remote_mode = 22;		
	
	//左右脑袋目标识别判断
	if(vision_receive.L_chase_pitch && vision_receive.L_chase_yaw)
	{
		Sentry.L_Flag_foe = 1;	//目标识别标志位
	}
	else
	{
		Sentry.L_Flag_foe = 0;
	}
	
	if(vision_receive.R_chase_pitch && vision_receive.R_chase_yaw)
	{
		Sentry.R_Flag_foe = 1;
	}
	else
	{
		Sentry.R_Flag_foe = 0;
	}
	
  //自瞄触发，注意只触发置位一次
	if((Sentry.L_Flag_foe == 1 || Sentry.R_Flag_foe == 1) && Sentry.Flag_mode==0)
	{
		Sentry.Flag_mode = 1;	//等候响应模式
		Sentry.L_Flag_pitch_direction = 0;	//关闭巡航
		Sentry.L_Flag_yaw_direction = 0;
		Sentry.R_Flag_pitch_direction = 0;
		Sentry.R_Flag_yaw_direction = 0;
	}
	
	//均丢失目标恢复巡航标志位
	if((Sentry.L_Flag_foe == 0 && Sentry.R_Flag_foe == 0) && Sentry.Flag_mode!=0)
	{
		Sentry.Flag_mode = 0;
	}

	//巡航恢复
	if(Sentry.Flag_mode == 0)
	{
		if(Sentry.L_Flag_pitch_direction == 0)	//注意需要判断，因为只能触发一次
		{
			Sentry.L_Flag_pitch_direction = 1;
		}
		if(Sentry.L_Flag_yaw_direction == 0)
		{
			Sentry.L_Flag_yaw_direction = 1;
		}
		if(Sentry.R_Flag_pitch_direction == 0)
		{
			Sentry.R_Flag_pitch_direction = 1;
		}
		if(Sentry.R_Flag_yaw_direction == 0)
		{
			Sentry.R_Flag_yaw_direction = 1;
		}
	}
	
}

//================================================哨兵状态及裁判系统数据初始化================================================//
static void Sentry_Init()
{
	Sentry.Flag_mode = 0;
	Sentry.L_Flag_foe = 0;
	Sentry.R_Flag_foe = 0;	
	Sentry.Flag_progress = 0;
	Sentry.Flag_judge = 0;
	Sentry.L_Flag_yaw_direction = 1;
	Sentry.R_Flag_yaw_direction = 1;
	Sentry.L_Flag_pitch_direction = 1;
	Sentry.R_Flag_pitch_direction = 1;
}

static void Vision_Init()
{
	vision.header = 0x5A;
	vision.L_pitch = Gimbal_left;
	vision.L_yaw = Yaw_left_c;
	vision.R_pitch = Gimbal_right;
	vision.R_yaw = Yaw_right_c;
	vision.ending = 0xAA;
}
