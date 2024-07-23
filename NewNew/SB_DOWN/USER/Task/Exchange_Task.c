#include "Exchange_Task.h"
#include "cmsis_os.h"
#include "uart_user.h"
#include "CRC.h"
#include "string.h"
#include "judge.h"
#include "can_user.h"

uint8_t Tx[128];
uint8_t Tx_friction[8];
uint8_t Tx_shijue[8];
uint8_t Tx_R_yaw[8];
uint8_t Rx_flag = 0; //程序初始化标志位
uint16_t checksum;
Tx_naving Tx_nav;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern uint8_t Rx[128];
extern uint32_t length; //DMA中未传输的数据个数
extern int count;
extern Sentry_t Sentry;
extern float L_yaw;	
extern float L_pitch;
extern float R_yaw;	
extern float R_pitch;
extern uint16_t time_delay;
extern uint8_t flag_suo;
extern double yaw12; //云台陀螺仪yaw值
extern uint8_t flag_heart;
extern Rx_naving Rx_nav;
extern float R_yaw_speed;

void Exchange_Task(void const * argument)
{
  for(;;)
  {
		JudgeSend(Rx_nav.sentry_decision, Datacmd_Decision);
		if(Rx_flag==0) //程序初始化标志位
		{
			RS485_Trans();
			Rx_flag = 1;
		}
		else if(Rx_flag == 2) //收到一帧数据后
		{
			RS485_Trans();
			Rx_flag = 1;
		}
		else
		{
			count++; //收到一帧数据后置零
		}
		
		if(count>=20) //如果有15ms没有收到导航数据
		{
			RS485_Trans();
			count = 0;
		}
		
		memset(Tx_friction, 0, sizeof(Tx_friction));//接收前清空数组
		//先高八位，再低八位
		Tx_friction[0] = (Sentry.Myself_17mm_heat_id1 >> 8) & 0xff; //枪管1实时热量
		Tx_friction[1] = Sentry.Myself_17mm_heat_id1 & 0xff;
		Tx_friction[2] = (Sentry.Myself_17mm_heat_id2 >> 8) & 0xff; //枪管2实时热量
		Tx_friction[3] = Sentry.Myself_17mm_heat_id2 & 0xff;
		Tx_friction[4] = flag_heart; //受伤装甲板ID
		memcpy(&Tx_friction[5], &time_delay, 2);
		Tx_friction[7] = flag_suo;
		can_remote(Tx_friction, 0x36);
    osDelay(1);
		
		float bullet_speed = Sentry.bullet_speed;
		memset(Tx_friction, 0, sizeof(Tx_friction));//接收前清空数组
		Tx_friction[0] = Sentry.shooter_ID; //枪管ID
		Tx_friction[1] = Sentry.bullet_frequence;	//枪管弹频
		memcpy(&Tx_friction[2], &bullet_speed, 4); //枪管射速
		Tx_friction[6] = 0;
		Tx_friction[7] = 0;
		can_remote(Tx_friction, 0x37);
		osDelay(1);
		
		memset(Tx_shijue, 0, sizeof(Tx_shijue));//接收前清空数组
		Tx_shijue[0] = Rx_nav.R_tracking; //从RS485收到的右头视觉数据，发到上板
		Tx_shijue[1] = Rx_nav.R_shoot;
		Tx_shijue[2] = Rx_nav.target_shijue;
		Tx_shijue[3] = Rx_nav.Flag_turn;
		can_remote(Tx_shijue, 0x39);
		osDelay(1);
		
		memset(Tx_R_yaw, 0, sizeof(Tx_R_yaw));//接收前清空数组
		memcpy(&Tx_R_yaw[0], &Rx_nav.R_yaw, 4); //从RS485收到的右头目标值数据，发到上板
		memcpy(&Tx_R_yaw[4], &Rx_nav.R_pitch, 4);
		can_remote(Tx_R_yaw, 0x40);
		osDelay(1);
  }
}

void RS485_Trans(void)
{
	Tx_nav.header = 0x5A;
	if(Sentry.Flag_progress == 0x04) //比赛中
	{
		Tx_nav.Flag_progress = 1;
	}
	else
	{
		Tx_nav.Flag_progress = 0;
	}
	if(Sentry.Flag_progress == 0x02) //15s自检阶段
	{
		Tx_nav.Flag_start = 1;
	}
	else
	{
		Tx_nav.Flag_start = 0;
	}
	Tx_nav.Flag_off_war = Sentry.off_war;
	Tx_nav.color = Sentry.Flag_judge; //1 red 2 blue
	Tx_nav.projectile_allowance_17mm = Sentry.projectile_allowance_17mm;
	Tx_nav.remaining_gold_coin = Sentry.remaining_gold_coin;
	Tx_nav.supply_robot_id = Sentry.supply_robot_id;
	Tx_nav.supply_projectile_num = Sentry.supply_projectile_num;
	Tx_nav.red_7_HP = Sentry.red_remain_HP;
	Tx_nav.red_outpost_HP = Sentry.red_outpost_HP;
	Tx_nav.red_base_HP = Sentry.red_base_HP;
	Tx_nav.blue_7_HP = Sentry.blue_remain_HP;
	Tx_nav.blue_outpost_HP = Sentry.blue_outpost_HP;
	Tx_nav.blue_base_HP = Sentry.blue_base_HP;
	
	Tx_nav.yaw12 = yaw12; //当前大yaw、右头yaw、pitch数值
	Tx_nav.R_yaw = R_yaw;
	Tx_nav.R_pitch = R_pitch;
	
	Tx_nav.tar_pos_x = Sentry.target_position_x;
	Tx_nav.tar_pos_y = Sentry.target_position_y;
	Tx_nav.cmd_key = Sentry.cmd_keyboard;
	Tx_nav.bullet_speed = Sentry.bullet_speed;
	Tx_nav.R_yaw_speed = R_yaw_speed;
	
	Tx_nav.ending = 0xAA;
	
	Tx[0] = Tx_nav.header;
	Tx[1] = Tx_nav.Flag_progress;
	Tx[2] = Tx_nav.color;
	memcpy(&Tx[3], &Tx_nav.projectile_allowance_17mm, 2);
	memcpy(&Tx[5], &Tx_nav.remaining_gold_coin, 2);
	memcpy(&Tx[7], &Tx_nav.supply_robot_id, 1);
	memcpy(&Tx[8], &Tx_nav.supply_projectile_num, 1);
	memcpy(&Tx[9], &Tx_nav.red_7_HP, 2);
	memcpy(&Tx[11], &Tx_nav.red_outpost_HP, 2);
	memcpy(&Tx[13], &Tx_nav.red_base_HP, 2);
	memcpy(&Tx[15], &Tx_nav.blue_7_HP, 2);
	memcpy(&Tx[17], &Tx_nav.blue_outpost_HP, 2);
	memcpy(&Tx[19], &Tx_nav.blue_base_HP, 2);
	memcpy(&Tx[21], &Tx_nav.yaw12, 8);
	memcpy(&Tx[29], &Tx_nav.R_yaw, 4);
	memcpy(&Tx[33], &Tx_nav.R_pitch, 4);
	memcpy(&Tx[37], &Tx_nav.tar_pos_x, 4);
	memcpy(&Tx[41], &Tx_nav.tar_pos_y, 4);
	memcpy(&Tx[45], &Tx_nav.cmd_key, 1);
	memcpy(&Tx[46], &Tx_nav.bullet_speed, 4);
	memcpy(&Tx[50], &Tx_nav.Flag_start, 1);
	memcpy(&Tx[51], &Tx_nav.Flag_off_war, 1);
	memcpy(&Tx[52], &Tx_nav.R_yaw_speed, 4);
	Tx_nav.checksum = Get_CRC16_Check_Sum(Tx, 56, 0xffff);
	memcpy(&Tx[56], &Tx_nav.checksum, 2);
	Tx[58] = Tx_nav.ending;
	
	HAL_GPIO_WritePin(DIR_2_GPIO_Port,DIR_2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_1_GPIO_Port,DIR_1_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit_IT(&huart1,Tx,sizeof(Tx));
	osDelay(5);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) //在 uart_user.c 中进行接收
{
    if(huart->Instance == USART1)
    {
			HAL_GPIO_WritePin(DIR_2_GPIO_Port,DIR_2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIR_1_GPIO_Port,DIR_1_Pin,GPIO_PIN_RESET);
    }
		if(huart == &huart5)
		{
			clear_uart5_tx_dma_busy_sign();
		}
}
