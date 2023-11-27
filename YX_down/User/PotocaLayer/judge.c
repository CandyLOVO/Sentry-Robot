#include "judge.h"
#include "CRC.h"
#include "main.h"
#include "struct_typedef.h"
#include "drv_can.h"


//=======================================================裁判系统校验读取(以中断形式)===============================================================//
extern uint8_t first_x;
extern uint8_t first_y;

//定义2个结构体，一个官方接收的，一个自己使用的
JUDGE_MODULE_DATA Judge_Hero;
Sentry_t Sentry;

static void Update_data();//更新定义一些需要用到的变量并实时更新数值方便其他文件调用（主要在这个函数里面修改）
	
void JUDGE_Receive(volatile uint8_t *databuffer,uint8_t length)
{
    uint8_t pos=0;
    uint16_t data_length=0;
    uint16_t CMD_ID =0;
    
     while(pos<length)
     {
        if(databuffer[pos]==0xA5)
        {
            if(Verify_CRC8_Check_Sum(&databuffer[pos],5))
            {
                data_length = (databuffer[pos+1]&0xff)|((databuffer[pos+2]<<8)&0xff00);
                if(pos+data_length+9>length)
                {
                    continue;
                }
            if(Verify_CRC16_Check_Sum(&databuffer[pos],data_length+9))
            {
              
             
                CMD_ID = (databuffer[pos+5]&0xff)|((databuffer[pos+6]<<8)&0xff00);
                switch(CMD_ID)
                { 
                    case 0x0001:
                        data_length = 11;
                        memcpy((void*)(&Judge_Hero.status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0003:
                        data_length = 32;
                         memcpy((void*)(&Judge_Hero.robot_hp), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0005:
                        data_length = 13;
                          memcpy((void*)(&Judge_Hero.zone), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0101:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.event_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0102:
                        data_length = 4;
                         memcpy((void*)(&Judge_Hero.supply_status), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0104 :
                        data_length = 2;
                        memcpy((void*)(&Judge_Hero.warning), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0105 :
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.dart_remaining_time), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0201:
                         data_length = 27;
                         memcpy((void*)(&Judge_Hero.robot_status), (const void*)(&databuffer[pos+7]), data_length);   //底盘功率限制上限在这
                        break;
                    case 0x0202:
                        data_length = 16;
                        memcpy((void*)(&Judge_Hero.power_heat), (const void*)(&databuffer[pos+7]), data_length);      //含实时功率热量数据
                        break;
                    case 0x0203:
                        data_length = 16;
                         memcpy((void*)(&Judge_Hero.robot_pos), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0204:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.buff), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0205:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.aerial_energy), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0206:
                        data_length =1;
                        memcpy((void*)(&Judge_Hero.robot_hurt), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0207:
                        data_length = 7;
                        memcpy((void*)(&Judge_Hero.shoot_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0208:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.bullet_remain), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0209:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.rfid_status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x020A:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.rfid_status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    default:break;
                }
                pos+=(data_length+9);
                continue;

            }


          }

        }

        pos++;
     
     }
		 //校验完成后
		 Update_data();
		 
}


static void Update_data()
{
//=======================================================各种上限状态(10HZ)===============================================================//
	Sentry.Myself_id = Judge_Hero.robot_status.robot_id;	//本机器人的ID
	Sentry.Myself_level = Judge_Hero.robot_status.robot_level;		//本机器人的等级
	Sentry.Myself_max_HP = Judge_Hero.robot_status.max_HP;	//本机器人血量上限
	Sentry.Myself_chassis_power_limit = Judge_Hero.robot_status.chassis_power_limit;	//底盘功率上限
	Sentry.Myself_42mm_speed_limit = Judge_Hero.robot_status.shooter_id1_42mm_speed_limit;	//本机器人42mm射速限制(没用)
	//枪管1
	Sentry.Myself_17mm_speed_limit_id1 = Judge_Hero.robot_status.shooter_id1_17mm_speed_limit;	//本机器人17mm射速限制
	Sentry.Myself_17mm_cooling_limit_id1 = Judge_Hero.robot_status.shooter_id1_17mm_cooling_limit;		//本机器人17mm热量上限
	Sentry.Myself_17mm_cooling_rate_id1 = Judge_Hero.robot_status.shooter_id1_17mm_cooling_rate;	//本机器人17mm热量每秒冷却值
	//枪管2
	Sentry.Myself_17mm_speed_limit_id2 = Judge_Hero.robot_status.shooter_id2_17mm_speed_limit;	//本机器人17mm射速限制
	Sentry.Myself_17mm_cooling_limit_id2 = Judge_Hero.robot_status.shooter_id2_17mm_cooling_limit;		//本机器人17mm热量上限
	Sentry.Myself_17mm_cooling_rate_id2 = Judge_Hero.robot_status.shooter_id2_17mm_cooling_rate;	//本机器人17mm热量每秒冷却值
	
//=======================================================实时数据(50HZ)===============================================================//
	Sentry.Myself_chassis_power_buffer = Judge_Hero.power_heat.chassis_power_buffer;	//实时缓冲能量
	Sentry.Myself_chassis_power = Judge_Hero.power_heat.chassis_power;	//实时底盘功率
	
	Sentry.Myself_17mm_speed_id1 = Judge_Hero.power_heat.shooter_id1_17mm_cooling_heat;	//实时枪管1热量
	Sentry.Myself_17mm_speed_id2 = Judge_Hero.power_heat.shooter_id2_17mm_cooling_heat;	//实时枪管2热量
	
	Sentry.Myself_remain_HP = Judge_Hero.robot_status.remain_HP;	//本机器人剩余血量（10HZ）
	Sentry.bullet_frequence = Judge_Hero.shoot_data.bullet_freq;	//实时射频（单位为HZ）
	Sentry.bullet_speed = Judge_Hero.shoot_data.bullet_speed;	//实时射速(单位m/s)
	Sentry.armor_id = Judge_Hero.robot_hurt.armor_id;		//受伤的装甲板编号（应该0是非装甲板受伤，1-4是装甲板伤害，需测试验证）
	Sentry.hurt_type = Judge_Hero.robot_hurt.hurt_type;	//受伤类型

	//比赛进程
	Sentry.Flag_progress =  Judge_Hero.status.game_progress;	//比赛进程,1为准备阶段，2为自检，3为倒计时，4为对战阶段，5为比赛结束(结算时)
	Sentry.Time_remain = Judge_Hero.status.stage_remain_time;		//比赛剩余时间

	//判断我方是红方还是蓝方（数字针对我是哨兵）
	if(Sentry.Myself_id == 7)//红色方
	{
		Sentry.Flag_judge = 1;
	}
	else if(Sentry.Myself_id == 107)
	{
		Sentry.Flag_judge = 2;
	}
	
	
	//	if(Sentry.Flag_progress == 4 && Sentry.Flag_first == 0)		//比赛开始，不能导航时才使用的开环方案
//	{
//		first_x = 1;
//		first_y = 1;
//		Sentry.Flag_first = 1;
//		HAL_TIM_Base_Start_IT(&htim8);
//	}
	
	//判断是红方哨兵还是蓝方,掉血强制开启旋转模式（血量已发生变化）
//	if(Sentry.Flag_judge == 1)//红色方
//	{
//		if(Judge_Hero.robot_hp.red_7_robot_HP!= 0 && Judge_Hero.robot_hp.red_7_robot_HP != 600)
//		{
//			Sentry.Flag_first = 2;
//		}
//	}
//	
//	else if(Sentry.Flag_judge == 2)//蓝色方
//	{
//		if(Judge_Hero.robot_hp.blue_7_robot_HP!= 0 && Judge_Hero.robot_hp.blue_7_robot_HP != 600)
//		{
//			Sentry.Flag_first = 2;
//		}
//	}
	
	//发送一些需要的信息给上C板
	uint8_t temp_remote[2];
	temp_remote[0] = Sentry.Flag_progress;
	temp_remote[1] = Sentry.Flag_judge;
	
	CAN_TxHeaderTypeDef tx_header;
    
  tx_header.StdId = 0x10;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 2;		//发送数据长度（字节）

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, temp_remote,(uint32_t*)CAN_TX_MAILBOX0);
}