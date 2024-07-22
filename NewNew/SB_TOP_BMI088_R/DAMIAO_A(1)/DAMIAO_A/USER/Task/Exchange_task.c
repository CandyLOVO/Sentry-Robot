#include "Exchange_task.h"
#include "imu_temp_control_task.h"
#include "Can_user.h"

//================================================ȫ�ֱ���================================================//
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
volatile uint8_t rx_len_uart4 = 0;  //����һ֡���ݵĳ���
volatile uint8_t recv_end_flag_uart4 = 0; //һ֡���ݽ�����ɱ�־

volatile uint8_t rx_len_uart5 = 0;  //����һ֡���ݵĳ���
volatile uint8_t recv_end_flag_uart5 = 0; //һ֡���ݽ�����ɱ�־

uint8_t rx_buffer_L[100]={0};  //�������ݻ�������
uint8_t rx_buffer_R[100]; 
uint8_t vision_send_L[100];	//�Ӿ��ӿڷ�������֡
uint8_t vision_send_R[100];

Vision_t vision;	//�Ӿ����ݷ��ͽṹ��
Vision_receive_t vision_receive;	//�Ӿ����ݽ��սṹ��
remote_flag_t remote;	//���̰�����ȡ(�ṹ��)
Sentry_t Sentry;	//�ڱ�״̬���Ͳ���ϵͳ���ݽṹ��
extern int error_uart_4;
extern int error_uart_5;
//================================================ȫ�ֱ���================================================//

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
