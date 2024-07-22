#include "Exchange_task.h"
#include "imu_temp_control_task.h"
#include "Can_user.h"

//================================================全局变量================================================//
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
volatile uint8_t rx_len_uart4 = 0;  //接收一帧数据的长度
volatile uint8_t recv_end_flag_uart4 = 0; //一帧数据接收完成标志

volatile uint8_t rx_len_uart5 = 0;  //接收一帧数据的长度
volatile uint8_t recv_end_flag_uart5 = 0; //一帧数据接收完成标志

uint8_t rx_buffer_L[100]={0};  //接收数据缓存数组
uint8_t rx_buffer_R[100]; 
uint8_t vision_send_L[100];	//视觉接口发送数据帧
uint8_t vision_send_R[100];

Vision_t vision;	//视觉数据发送结构体
Vision_receive_t vision_receive;	//视觉数据接收结构体
remote_flag_t remote;	//键盘按键读取(结构体)
Sentry_t Sentry;	//哨兵状态量和裁判系统数据结构体
extern int error_uart_4;
extern int error_uart_5;
//================================================全局变量================================================//

uint8_t Tx_gyro[4];

extern fp32 gyro[3];

void Exchange_task(void const * argument)
{
  for(;;)
  {
		memcpy(&Tx_gyro[0], &gyro[2], 4);
		can_remote(Tx_gyro, 0x52);
		osDelay(1);
  }
} 
