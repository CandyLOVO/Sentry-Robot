#include "Exchange_task.h"

//================================================通信任务================================================//

//针对哨兵
uint8_t foe_flag = 0;		//视觉检测标志位
uint8_t foe_count = 0;
uint8_t Flag_progress;
uint8_t Flag_judge = 0;

uint8_t vision_send[28];	//视觉接口数据帧
remote_flag_t remote;	//键盘按键读取

extern RC_ctrl_t rc_ctrl;
extern uint16_t Remember_pitch;
extern fp32 Err_pitch;
extern uint8_t Remember_pitch_flag;
extern fp32 ins_yaw;
uint8_t ins_buf[8];
uint16_t ins_buf_temp;		//define a temp to receive and change float to int16
 

#define BUFFER_SIZE 100
extern UART_HandleTypeDef huart1;
volatile uint8_t rx_len_uart1 = 0;  //接收一帧数据的长度
volatile uint8_t recv_end_flag_uart1 = 0; //一帧数据接收完成标志
uint8_t rx_buffer[100]={0};  //接收数据缓存数组
int16_t Yaw_minipc;
int16_t Pitch_minipc;
fp32 Yaw_minipc_fp;
fp32 Pitch_minipc_fp;
Vision_t vision;	//视觉通信结构体

void Get_keyboard();
void Get_minipc();
void remote_data_read(uint8_t rx_buffer[]);
void Stm_pc_send();

void Exchange_task(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	
	ins_buf[0] = 8;	//imu receive tag
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能uart1的IDLE中断
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100); //开启接收
	
  for(;;)
  {
		ins_buf_temp = ins_yaw + 180;		// Add 180 to be a positive number
		ins_buf[1] = ins_buf_temp;
		ins_buf[2] = ins_buf_temp >> 8;		// 2 bytes put together forming a uint_16
		can_remote(ins_buf,0x55);
		
		Get_keyboard();		//解算keyboard flag	
		Get_minipc();
		Stm_pc_send();
		if(Yaw_minipc_fp || Pitch_minipc_fp)	//刚好在正中心不知道会不会出现抖动
		{
			foe_flag = 1;
			foe_count = 0;
		}

    osDelay(1);
  }
  /* USER CODE END StartTask03 */
} 

//================================================获取键盘数据================================================//
void Get_keyboard()	
{
		memcpy(&remote.key , &rc_ctrl.key , 2);
		remote.mouse.press_left = rc_ctrl.mouse.press_l;
		remote.mouse.press_right = rc_ctrl.mouse.press_r;
	  remote.mouse.x = rc_ctrl.mouse.x;
		remote.mouse.y = rc_ctrl.mouse.y;
}

//================================================通信接收任务================================================//
void Get_minipc()
{

		if(recv_end_flag_uart1 == 1)  //接收完成标志
		{			
			if(rx_buffer[0] == 0x01)
			{
				remote_data_read(rx_buffer);
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

//================================================通信解算任务================================================//
void remote_data_read(uint8_t rx_buffer[])
{
	Yaw_minipc = (int)(rx_buffer[1] << 8 | rx_buffer[2]);
	Pitch_minipc = (int)(rx_buffer[3] << 8 | rx_buffer[4]);
	Yaw_minipc_fp = (float)(Yaw_minipc * 100)/32767;
	Pitch_minipc_fp = (float)(Pitch_minipc * 100)/32767;	
	Yaw_minipc = (int)(Yaw_minipc * 100)/32767;
	Pitch_minipc = (int)(Pitch_minipc * 100)/32767;	
}

//================================================数据stm32 -> 上位机================================================//
void Stm_pc_send()
{
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);

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
	memcpy(&vision_send[26],&vision.checksum,2);
	
	HAL_UART_Transmit_DMA(&huart1,vision_send,28);
}

