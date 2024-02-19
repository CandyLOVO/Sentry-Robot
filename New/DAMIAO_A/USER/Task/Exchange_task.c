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
extern UART_HandleTypeDef huart1;
volatile uint8_t rx_len_uart1 = 0;  //接收一帧数据的长度
volatile uint8_t recv_end_flag_uart1 = 0; //一帧数据接收完成标志
uint8_t rx_buffer[100]={0};  //接收数据缓存数组
uint8_t vision_send[100];	//视觉接口发送数据帧

Vision_t vision;	//视觉数据发送结构体
vision_receive_t vision_receive;	//视觉数据接收结构体
remote_flag_t remote;	//键盘按键读取(结构体)
Sentry_t Sentry;	//哨兵状态量和裁判系统数据结构体


void Exchange_task(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	Vision_Init();
	Sentry_Init();	//哨兵状态量及裁判系统数据初始化
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能uart1的IDLE中断
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100); //开启接收
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

//================================================通信接收任务（未使用，已经移植到中断中）================================================//
static void Get_minipc()
{
		if(recv_end_flag_uart1 == 1)  //接收完成标志
		{			
			if(rx_buffer[0] == 0xA5)
			{
				Vision_read(rx_buffer);
			}
			
			recv_end_flag_uart1 = 0;//清除接收结束标志位
			for(uint8_t i=0;i<rx_len_uart1;i++)
				{
					rx_buffer[i]=0;//清接收缓存
				}
				//memset(rx_buffer,0,rx_len);
			rx_len_uart1 = 0;//清除计数
			HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);//重新打开DMA接收			
		}
}

//================================================通信读取解算任务================================================//
void Vision_read(uint8_t rx_buffer[])
{
	memcpy(&vision_receive.header,&rx_buffer[0],1);
	memcpy(&vision_receive.L_chase_pitch,&rx_buffer[1],4);
	memcpy(&vision_receive.L_chase_yaw,&rx_buffer[5],4);
	memcpy(&vision_receive.R_chase_pitch,&rx_buffer[9],4);
	memcpy(&vision_receive.R_chase_yaw,&rx_buffer[13],4);
	memcpy(&vision_receive.naving,&rx_buffer[17],1);
	memcpy(&vision_receive.nav_vx,&rx_buffer[18],4);
	memcpy(&vision_receive.nav_vy,&rx_buffer[22],4);
	memcpy(&vision_receive.checksum,&rx_buffer[26],2);
}

//================================================数据stm32 -> 上位机================================================//
static void Stm_pc_send()
{
	//更新姿态数据
	vision.L_pitch = Gimbal_left;
	vision.L_yaw = Yaw_left_c;
	vision.R_pitch = Gimbal_right;
	vision.R_yaw = Yaw_right_c;
	
	memcpy(&vision_send[0],&vision.header,1);
	memcpy(&vision_send[1],&vision.L_pitch,4);
	memcpy(&vision_send[5],&vision.L_yaw,4);
	memcpy(&vision_send[9],&vision.R_pitch,4);
	memcpy(&vision_send[13],&vision.R_yaw,4);
	memcpy(&vision_send[17],&Sentry.Flag_mode,1);//哨兵目前的模式
	memcpy(&vision_send[18],&Sentry.Flag_progress,1);//裁判系统比赛进程数据
	memcpy(&vision_send[19],&Sentry.Flag_judge,1);//红蓝方检测，置0为裁判系统寄了，置1为我方是红色方，置2为我方是蓝色方
	memcpy(&vision_send[20],&vision.checksum,2);
	
	HAL_UART_Transmit_DMA(&huart1,vision_send,22);
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
	//发送导航数据
		uint8_t ins_buf[8] = {0};
		ins_buf[0] = 8;	//	imu头帧标识
		memcpy(&ins_buf[1],&Yaw_middle_c,4); //获取yaw的角度并储存在发送的字节中
		memcpy(&ins_buf[5],&vision_receive.naving,1);
		can_remote(ins_buf,0x55);
		osDelay(1);
		
		uint8_t ins_buf1[8] = {0};
		ins_buf1[0] = 9;	//	imu头帧标识
		memcpy(&ins_buf1[1],&vision_receive.nav_vx,4);
		can_remote(ins_buf1,0x56);
		osDelay(1);
		
		uint8_t ins_buf2[8] = {0};
		ins_buf2[0] = 10;	//	imu头帧标识
		memcpy(&ins_buf2[1],&vision_receive.nav_vy,4);
		can_remote(ins_buf2,0x57);
		osDelay(1);
}

//================================================通过上位机数据更新哨兵状态结构体================================================//
//判断上位机检测到目标，检测到就进行解算，没检测到赋0
static void Judge_minipc()
{

}

//================================================哨兵状态及裁判系统数据初始化================================================//
static void Sentry_Init()
{
	Sentry.Flag_mode = 0;
	Sentry.L_Flag_foe = 0;
	Sentry.R_Flag_foe = 0;	
	Sentry.Flag_progress = 0;
	Sentry.Flag_judge = 0;
}

static void Vision_Init()
{
	vision.header = 0x5A;
	vision.L_pitch = Gimbal_left;
	vision.L_yaw = Yaw_left_c;
	vision.R_pitch = Gimbal_right;
	vision.R_yaw = Yaw_right_c;
	vision.checksum = 0xAAAA;	//CRC16校验，我没用，发了个定值做校验	
}