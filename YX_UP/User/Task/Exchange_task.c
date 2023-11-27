#include "Exchange_task.h"

//================================================通信任务================================================//


//================================================函数================================================//
//上C板向下C板发送数据
static void Up_send_to_down();

//获取键盘信息
static void Get_keyboard();

//获取Nuc的信息
static void Get_minipc();

//判断上位机检测到目标，检测到就进行解算，没检测到赋0
static void Judge_minipc();

//对来自Nuc的信息进行解码
static void Vision_read(uint8_t rx_buffer[]);

//向Nuc发送信息
static void Stm_pc_send();

//弹道补偿API接口初始化
static void SolveTrajectory_Init();

//哨兵状态量及官方裁判系统数据初始化
static void Sentry_Init();

//================================================全局变量================================================//
extern UART_HandleTypeDef huart1;
volatile uint8_t rx_len_uart1 = 0;  //接收一帧数据的长度
volatile uint8_t recv_end_flag_uart1 = 0; //一帧数据接收完成标志
uint8_t rx_buffer[100]={0};  //接收数据缓存数组
uint8_t vision_send[28];	//视觉接口发送数据帧

volatile Chase_t chase;	//赋予电机追踪的数据结构体
Vision_t vision;	//视觉数据发送结构体
vision_receive_t vision_receive;	//视觉数据接收结构体
remote_flag_t remote;	//键盘按键读取(结构体)
Sentry_t Sentry;	//哨兵状态量和裁判系统数据结构体


void Exchange_task(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
	Sentry_Init();	//哨兵状态量及裁判系统数据初始化
	SolveTrajectory_Init();//初始化参数
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能uart1的IDLE中断
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100); //开启接收
	
  for(;;)
  {
		osDelay(1);
		Up_send_to_down();	//上C向下C发送信息
		Get_keyboard();		//解算键盘的信息
		Get_minipc();		//取出nuc的信息
		Judge_minipc();		//检测是否识别到目标，识别到目标就解算，没识别到赋0
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

//================================================通信接收任务================================================//
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
static void Vision_read(uint8_t rx_buffer[])
{
	memcpy(&vision_receive.official,&rx_buffer[1],1); 
	memcpy(&vision_receive.x,&rx_buffer[2],4); 
	memcpy(&vision_receive.y,&rx_buffer[6],4); 
	memcpy(&vision_receive.z,&rx_buffer[10],4); 
	memcpy(&vision_receive.yaw,&rx_buffer[14],4); 
	memcpy(&vision_receive.vx,&rx_buffer[18],4); 
	memcpy(&vision_receive.vy,&rx_buffer[22],4); 
	memcpy(&vision_receive.vz,&rx_buffer[26],4); 
	memcpy(&vision_receive.v_yaw,&rx_buffer[30],4); 
	memcpy(&vision_receive.r1,&rx_buffer[34],4); 
	memcpy(&vision_receive.r2,&rx_buffer[38],4);
	memcpy(&vision_receive.dz,&rx_buffer[42],4);
	memcpy(&vision_receive.checksum,&rx_buffer[44],2);
	
	st.tar_yaw = vision_receive.yaw;
	st.v_yaw = vision_receive.v_yaw;
	st.armor_num = 2;
	st.xw = vision_receive.x;
	st.yw = vision_receive.y;
	st.zw = vision_receive.z;
	st.r1 = vision_receive.r1;
	st.r2 = vision_receive.r2;
	st.dz = vision_receive.dz;
	st.vxw = vision_receive.vx;
	st.vyw = vision_receive.vy;
	st.vzw = vision_receive.vz;
	st.current_v = 28;
}

//================================================数据stm32 -> 上位机================================================//
static void Stm_pc_send()
{
	vision.header = 0x5A;
	vision.official.detect_color = 1;	//读取裁判系统数据判断红蓝方
	vision.official.reset_tracker = 0;
	vision.official.reserved = 6;
	vision.roll = INS_angle[2]/57.3f;
	vision.pitch = INS_angle[1]/57.3f;
	vision.yaw = INS_angle[0]/57.3f;
	vision.aim_x = 0.5;
	vision.aim_y = 0.5;
	vision.aim_z = 5;
	vision.checksum = 0xAAAA;	//CRC16校验，我没用，发了个定值做校验
	
	memcpy(&vision_send[0],&vision.header,1);
	memcpy(&vision_send[1],&vision.official,1);
	memcpy(&vision_send[2],&vision.roll,4);
	memcpy(&vision_send[6],&vision.pitch,4);
	memcpy(&vision_send[10],&vision.yaw,4);
	memcpy(&vision_send[14],&vision.aim_x,4);
	memcpy(&vision_send[18],&vision.aim_y,4);
	memcpy(&vision_send[22],&vision.aim_z,4);
	memcpy(&vision_send[26],&vision.checksum,1);
	
	HAL_UART_Transmit_DMA(&huart1,vision_send,27);
}

//================================================弹道补偿API接口================================================//
static void SolveTrajectory_Init()
{
		//    //定义参数
    st.k = 0.092;
    st.bullet_type =  BULLET_17;
    st.current_v = 18;
    st.current_pitch = 0;
    st.current_yaw = 0;
    st.xw = 3.0;
    // st.yw = 0.0159;
    st.yw = 0;
    // st.zw = -0.2898;
    st.zw = 1.5;

    st.vxw = 0;
    st.vyw = 0;
    st.vzw = 0;
    st.v_yaw = 0;
    st.tar_yaw = 0.09131;
    st.r1 = 0.5;
    st.r2 = 0.5;
    st.dz = 0.1;
    st.bias_time = 100;
    st.s_bias = 0.00;
    st.z_bias = -0.08;
    st.armor_id = ARMOR_INFANTRY3;
    st.armor_num = 2;//ARMOR_NUM_NORMAL;
		
		//初始化vision
		vision.header = 0x5A;
		vision.official.detect_color = 1;	//读取裁判系统数据判断红蓝方
		vision.official.reset_tracker = 0;
		vision.official.reserved = 6;
		vision.roll = INS_angle[2]/57.3f;
		vision.pitch = INS_angle[1]/57.3f;
		vision.yaw = INS_angle[0]/57.3f;
		vision.aim_x = 0.5;
		vision.aim_y = 0.5;
		vision.aim_z = 5;
		vision.checksum = 0xAAAA;	//CRC16校验，我没用，发了个定值做校验
		
}

//================================================上C向下C发送数据================================================//
static void Up_send_to_down()
{
		uint8_t ins_buf[8] = {0};
		ins_buf[0] = 8;	//	imu头帧标识
		memcpy(&ins_buf[1],&INS_angle[0],4); //获取yaw的角度并储存在发送的字节中
		can_remote(ins_buf,0x55);
}

//================================================视觉识别检测判断================================================//
//判断上位机检测到目标，检测到就进行解算，没检测到赋0
static void Judge_minipc()
{
		if(vision_receive.x && vision_receive.y && vision_receive.z)
		{
			autoSolveTrajectory(&vision.pitch, &vision.yaw, &vision.aim_x, &vision.aim_y, &vision.aim_z);	//弹道解算
			chase.pitch = vision.pitch;
			chase.yaw = vision.yaw;
			
			Sentry.foe_flag = 1;	//识别标志位
			Sentry.foe_count = 0;	//计数器清零
			Sentry.Flag_shoot = 1;	//射击标识位（无暂留）
		}
		else
		{
			chase.pitch = target_pitch;
			chase.yaw = target_yaw;
			Sentry.Flag_shoot = 0;	//射击标识位（无暂留）
		}
}

//================================================哨兵状态及裁判系统数据初始化================================================//
static void Sentry_Init()
{
	Sentry.foe_flag = 0;
	Sentry.foe_count = 0;
	Sentry.Flag_progress = 0;
	Sentry.Flag_judge = 0;
}