#include "judge.h"
#include "CRC.h"
#include "main.h"
#include "struct_typedef.h"
#include "string.h"

//=======================================================裁判系统校验读取(以中断形式)===============================================================//
extern uint8_t first_x;
extern uint8_t first_y;

//定义2个结构体，一个官方接收的，一个自己使用的
JUDGE_MODULE_DATA Judge_Hero;
Sentry_t Sentry;
int32_t event;
int8_t data[4];

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
                        data_length = 3;
                        memcpy((void*)(&Judge_Hero.warning), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0105 :
                        data_length = 3;
                        memcpy((void*)(&Judge_Hero.dart_remaining_time_t), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0201:
                         data_length = 13;
                         memcpy((void*)(&Judge_Hero.robot_status), (const void*)(&databuffer[pos+7]), data_length);   //底盘功率限制上限在这
                        break;
                    case 0x0202:
                        data_length = 16;
                        memcpy((void*)(&Judge_Hero.power_heat), (const void*)(&databuffer[pos+7]), data_length);      //含实时功率热量数据
                        break;
                    case 0x0203:
                        data_length = 12;
                         memcpy((void*)(&Judge_Hero.robot_pos), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0204:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.buff), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0205:
                        data_length = 2;
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
                        memcpy((void*)(&Judge_Hero.rfid_status_t), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x020A:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.dart_client_cmd), (const void*)(&databuffer[pos+7]), data_length);
                        break;
										 case 0x020B:
                        data_length = 40;
                        memcpy((void*)(&Judge_Hero.ground_robot_position), (const void*)(&databuffer[pos+7]), data_length);
                        break;
										 case 0x020C:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.radar_mark_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
										 case 0x020D:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.sentry_information), (const void*)(&databuffer[pos+7]), data_length);
                        break;
										 case 0x020E:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.radar_information), (const void*)(&databuffer[pos+7]), data_length);
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

	
//=======================================================实时数据(50HZ)===============================================================//
	Sentry.Myself_chassis_power_buffer = Judge_Hero.power_heat.buffer_energy;	//实时缓冲能量
	Sentry.Myself_chassis_power = Judge_Hero.power_heat.chassis_power;	//实时底盘功率
	
	Sentry.Myself_17mm_speed_id1 = Judge_Hero.power_heat.shooter_17mm_1_barrel_heat;	//实时枪管1热量
	Sentry.Myself_17mm_speed_id2 = Judge_Hero.power_heat.shooter_17mm_2_barrel_heat;	//实时枪管2热量
	

	Sentry.shooter_ID = Judge_Hero.shoot_data.shooter_number; //发射机构ID，1-17mm枪管1，2-17mm枪管2，3-24mm枪管
	if(Sentry.shooter_ID==1)
	{
		Sentry.bullet_frequence_1 = Judge_Hero.shoot_data.launching_frequency;	//实时射频（单位为HZ）
		Sentry.bullet_speed_1 = Judge_Hero.shoot_data.initial_speed;	//实时射速(单位m/s)
	}
	if(Sentry.shooter_ID==2)
	{
		Sentry.bullet_frequence_2 = Judge_Hero.shoot_data.launching_frequency;	//实时射频（单位为HZ）
	  Sentry.bullet_speed_2 = Judge_Hero.shoot_data.initial_speed;	//实时射速(单位m/s)
	}
	
	Sentry.armor_id = Judge_Hero.robot_hurt.armor_id;		//受伤的装甲板编号（应该0是非装甲板受伤，1-4是装甲板伤害，需测试验证）
	Sentry.hurt_type = Judge_Hero.robot_hurt.HP_deduction_reason;	//血量变化类型
	
	//比赛进程
	Sentry.Flag_progress =  Judge_Hero.status.game_progress;	//比赛进程,1为准备阶段，2为自检，3为倒计时，4为对战阶段，5为比赛结束(结算时)
	Sentry.Time_remain = Judge_Hero.status.stage_remain_time;		//比赛剩余时间
	event = Judge_Hero.event_data.event_type;	
	memcpy(&data[0],&event,4);
	
	//判断我方是红方还是蓝方（数字针对我是哨兵）
	if(Sentry.Myself_id == 7)//红色方
	{
		Sentry.Flag_judge = 1;
		Sentry.Myself_remain_HP = Judge_Hero.robot_hp.red_7_robot_HP;
		Sentry.base_HP = Judge_Hero.robot_hp.blue_base_HP; //敌方基地血量
	}
	else if(Sentry.Myself_id == 107)
	{
		Sentry.Flag_judge = 2;
		Sentry.Myself_remain_HP = Judge_Hero.robot_hp.blue_7_robot_HP;
		Sentry.base_HP = Judge_Hero.robot_hp.red_base_HP; //敌方基地血量
	}
	Sentry.supply_projectile_num = Judge_Hero.supply_status.supply_projectile_num;
}
