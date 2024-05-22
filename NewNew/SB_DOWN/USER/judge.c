#include "judge.h"
#include "CRC.h"
#include "main.h"
#include "struct_typedef.h"
#include "string.h"
#include "cmsis_os.h"

#define REFEREE_SOF 0xA5 // 起始字节,协议固定为0xA5
#define ID_sentry_cmd 0x0120 // 哨兵自主决策

uint8_t UI_Seq; // 包序号，供整个referee文件使用
uint8_t Judge_Seq;
extern UART_HandleTypeDef huart5; //裁判系统 UART5

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
                        memcpy((void*)(&Judge_Hero.status), (const void*)(&databuffer[pos+7]), data_length); //比赛类型、当前比赛阶段、当前阶段剩余时间
                        break;
										case 0x0002:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.result), (const void*)(&databuffer[pos+7]), data_length); //胜方
                        break;
                    case 0x0003:
                        data_length = 32;
                         memcpy((void*)(&Judge_Hero.robot_hp), (const void*)(&databuffer[pos+7]), data_length); //红蓝方机器人及基地血量
                         break;
                    case 0x0101:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.event_data_t), (const void*)(&databuffer[pos+7]), data_length); //己方能量机关、高地占领状态
                        break;
                    case 0x0102:
                        data_length = 4;
                         memcpy((void*)(&Judge_Hero.supply_status), (const void*)(&databuffer[pos+7]), data_length); //补弹
                         break;
                    case 0x0104 :
                        data_length = 3;
                        memcpy((void*)(&Judge_Hero.warning), (const void*)(&databuffer[pos+7]), data_length); //判罚
                        break;
                    case 0x0105 :
                        data_length = 3;
                        memcpy((void*)(&Judge_Hero.dart_remaining_time_t), (const void*)(&databuffer[pos+7]), data_length); //飞镖
                         break;
                    case 0x0201:
                         data_length = 13;
                         memcpy((void*)(&Judge_Hero.robot_status), (const void*)(&databuffer[pos+7]), data_length); //机器人ID、等级、血量、血量上限、热量冷却速度、热量上限、功率上限、电源管理模块的输出情况
                        break;
                    case 0x0202:
                        data_length = 16;
                        memcpy((void*)(&Judge_Hero.power_heat), (const void*)(&databuffer[pos+7]), data_length); //电源管理模块的chassis口输出电压、电流、底盘功率、缓冲能量、发射机构的枪口热量
                        break;
                    case 0x0203:
                        data_length = 12;
                         memcpy((void*)(&Judge_Hero.robot_pos), (const void*)(&databuffer[pos+7]), data_length); //本机器人位置、测速模块朝向
                         break;
                    case 0x0204:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.buff), (const void*)(&databuffer[pos+7]), data_length); //机器人增益
                        break;
                    case 0x0205:
                        data_length = 2;
                        memcpy((void*)(&Judge_Hero.aerial_energy), (const void*)(&databuffer[pos+7]), data_length); //空中机器人
                        break;
                    case 0x0206:
                        data_length =1;
                        memcpy((void*)(&Judge_Hero.robot_hurt), (const void*)(&databuffer[pos+7]), data_length); //扣血原因、血量变化类型
                        break;
                    case 0x0207:
                        data_length = 7;
                        memcpy((void*)(&Judge_Hero.shoot_data), (const void*)(&databuffer[pos+7]), data_length); //发射机构、射速 HZ、弹丸初速度 m/s
                        break;
                    case 0x0208:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.bullet_remain), (const void*)(&databuffer[pos+7]), data_length); //允许发弹量、剩余金币数量
                        break;
                    case 0x0209:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.rfid_status_t), (const void*)(&databuffer[pos+7]), data_length); //是否已检测到该增益点RFID卡
                        break;
                    case 0x020A:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.dart_client_cmd), (const void*)(&databuffer[pos+7]), data_length); //飞镖发射站
                        break;
										 case 0x020B:
                        data_length = 40;
                        memcpy((void*)(&Judge_Hero.ground_robot_position), (const void*)(&databuffer[pos+7]), data_length); //己方机器人位置
                        break;
										 case 0x020C:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.radar_mark_data), (const void*)(&databuffer[pos+7]), data_length); //对方机器人被标记进度
                        break;
										 case 0x020D:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.sentry_information), (const void*)(&databuffer[pos+7]), data_length); //哨兵成功兑换发弹量、兑换发弹量次数、兑换血量次数
                        break;
										 case 0x020E:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.radar_information), (const void*)(&databuffer[pos+7]), data_length); //雷达
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
	Sentry.Myself_chassis_power_limit = Judge_Hero.robot_status.chassis_power_limit;
	Sentry.Myself_17mm_heat_id1 = Judge_Hero.power_heat.shooter_17mm_1_barrel_heat;	//实时枪管1热量
	Sentry.Myself_17mm_heat_id2 = Judge_Hero.power_heat.shooter_17mm_2_barrel_heat;	//实时枪管2热量
	

	Sentry.shooter_ID = Judge_Hero.shoot_data.shooter_number; //发射机构ID，1-17mm枪管1，2-17mm枪管2，3-24mm枪管
	Sentry.bullet_frequence = Judge_Hero.shoot_data.launching_frequency;	//弹丸射频（单位为HZ）
	Sentry.bullet_speed = Judge_Hero.shoot_data.initial_speed;	//弹丸初始射速(单位m/s)
	Sentry.projectile_allowance_17mm = Judge_Hero.bullet_remain.projectile_allowance_17mm; //17mm弹丸允许发弹量
	Sentry.remaining_gold_coin = Judge_Hero.bullet_remain.remaining_gold_coin; //剩余金币数量
	
	Sentry.supply_robot_id = Judge_Hero.supply_status.supply_robot_id; //补弹机器人ID
	Sentry.supply_projectile_num = Judge_Hero.supply_status.supply_projectile_num; //补弹数量
	
	Sentry.armor_id = Judge_Hero.robot_hurt.armor_id;		//受伤的装甲板编号（应该0是非装甲板受伤，1-4是装甲板伤害，需测试验证）
	Sentry.hurt_type = Judge_Hero.robot_hurt.HP_deduction_reason;	//血量变化类型
	
	//比赛进程
	Sentry.Flag_progress =  Judge_Hero.status.game_progress;	//比赛进程,1为准备阶段，2为自检，3为倒计时，4为对战阶段，5为比赛结束(结算时)
	Sentry.Time_remain = Judge_Hero.status.stage_remain_time;		//比赛剩余时间
	Sentry.event_data = Judge_Hero.event_data_t.event_data; //己方补给点、能量机关、高地占领状态、飞镖击中目标、中心增益点占领情况
	
	//判断我方是红方还是蓝方（数字针对我是哨兵）
	if(Sentry.Myself_id == 7)//红色方
	{
		Sentry.Flag_judge = 1; //判断红方的标志位
		Sentry.my_outpost_HP = Sentry.red_outpost_HP;
	}
	else if(Sentry.Myself_id == 107)
	{
		Sentry.Flag_judge = 2; //判断蓝方的标志位
		Sentry.my_outpost_HP = Sentry.blue_outpost_HP;
	}
	
	Sentry.red_remain_HP = Judge_Hero.robot_hp.red_7_robot_HP; //己方哨兵血量
	Sentry.red_outpost_HP = Judge_Hero.robot_hp.red_outpost_HP; //己方前哨战血量
	Sentry.red_base_HP = Judge_Hero.robot_hp.red_base_HP; //己方基地血量
	
	Sentry.blue_remain_HP = Judge_Hero.robot_hp.blue_7_robot_HP; //己方哨兵血量
	Sentry.blue_outpost_HP = Judge_Hero.robot_hp.blue_outpost_HP; //己方前哨战血量
	Sentry.blue_base_HP = Judge_Hero.robot_hp.blue_base_HP; //己方基地血量
	
	Sentry.rfid_status = Judge_Hero.rfid_status_t.rfid_status;
//	memcpy(&Sentry.rfid[0], &Sentry.rfid_status, 4);
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
//	HAL_UART_Transmit_DMA(&huart5, send, tx_len);
	HAL_UART_Transmit_DMA(&huart5, send, tx_len);
//	osDelay(115);
}

/************************************************UI推送字符（使更改生效）*********************************/
void JudgeSend(uint32_t TXData,uint16_t datacmdid)
{
	static UI_CharReFresh_t senddatatoJudge;

	uint8_t temp_datalength = Interactive_Data_LEN_Head + 32; // 计算交互数据长度

	senddatatoJudge.FrameHeader.SOF = REFEREE_SOF;
	senddatatoJudge.FrameHeader.DataLength = temp_datalength;
	senddatatoJudge.FrameHeader.Seq = Judge_Seq;
	senddatatoJudge.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&senddatatoJudge, LEN_CRC8, 0xFF);

	senddatatoJudge.CmdID =Cmdid_Judge_send;

	senddatatoJudge.datahead.data_cmd_id = datacmdid;

	senddatatoJudge.datahead.receiver_ID = 0x8080;
	senddatatoJudge.datahead.sender_ID = Sentry.Myself_id;
	
	int8_t txdata[4];
//	memcpy(&txdata[0], &TXData, 4);
	txdata[3]=TXData & 0xffff;
	txdata[2]=(TXData>>8) & 0xffff;
	txdata[1]=(TXData>>16) & 0xffff;
	txdata[0]=(TXData>>24) & 0xffff;
	
	for(int i=0; i<4; i++)
	{
		senddatatoJudge.String_Data[i] = txdata[i];
	}

	senddatatoJudge.frametail = Get_CRC16_Check_Sum((uint8_t *)&senddatatoJudge, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);

	RefereeSend((uint8_t *)&senddatatoJudge, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // 发送

	Judge_Seq++; // 包序号+1
}
