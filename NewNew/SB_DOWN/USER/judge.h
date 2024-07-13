#ifndef JUDGE_H
#define JUDGE_H

#include "uart_user.h"


#define Cmdid_Judge_send 0x0301  //裁判系统交互

#define Datacmd_Decision 0x0120  //哨兵自主决策指令
#define ID_Dataccenter	 0x8080 //裁判系统服务器id（用于哨兵和雷达自主决策指令）
typedef __packed struct
{
	uint8_t  SOF;						
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	 
}FrameHeader;    

typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef __packed struct
{
     uint8_t winner;
} ext_game_result_t;

typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

//typedef __packed struct
//{
// uint8_t F1_zone_status:1;
// uint8_t F1_zone_buff_debuff_status:3;
// uint8_t F2_zone_status:1;
// uint8_t F2_zone_buff_debuff_status:3;
// uint8_t F3_zone_status:1;
// uint8_t F3_zone_buff_debuff_status:3;
// uint8_t F4_zone_status:1;
// uint8_t F4_zone_buff_debuff_status:3;
// uint8_t F5_zone_status:1;
// uint8_t F5_zone_buff_debuff_status:3;
// uint8_t F6_zone_status:1;
// uint8_t F6_zone_buff_debuff_status:3;
// uint16_t red1_bullet_left;
// uint16_t red2_bullet_left;
//uint16_t blue1_bullet_left;
//uint16_t blue2_bullet_left;
//uint8_t lurk_mode;
//uint8_t res;
//} ext_ICRA_buff_debuff_zone_and_lurk_status_t;

typedef __packed struct
{
    uint32_t event_data;
} ext_event_data_t;

typedef __packed struct
{
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t; 

typedef __packed struct
{
    uint8_t level;
		uint8_t offending_robot_id;
		uint8_t count;
} ext_referee_warning_t;

typedef __packed struct
{
		uint8_t dart_remaining_time;
		uint16_t dart_info;
} dart_info_t;

typedef __packed struct
{
    uint8_t robot_id;
		uint8_t robot_level;
		uint16_t current_HP; 
		uint16_t maximum_HP;
		uint16_t shooter_barrel_cooling_value;
		uint16_t shooter_barrel_heat_limit;
		uint16_t chassis_power_limit; 
		uint8_t power_management_gimbal_output : 1;
		uint8_t power_management_chassis_output : 1; 
		uint8_t power_management_shooter_output : 1;
} ext_game_robot_status_t;

typedef __packed struct
{
		uint16_t chassis_voltage;
		uint16_t chassis_current;
		float chassis_power;
		uint16_t buffer_energy;
		uint16_t shooter_17mm_1_barrel_heat;
		uint16_t shooter_17mm_2_barrel_heat;
		uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

typedef __packed struct
{
		float x;
		float y;
		float angle;
} ext_game_robot_pos_t;

typedef __packed struct
{
		uint8_t recovery_buff;
		uint8_t cooling_buff;
		uint8_t defence_buff;
		uint8_t vulnerability_buff;
		uint16_t attack_buff;
} ext_buff_t;

typedef __packed struct
{	
		uint8_t airforce_status;
		uint8_t time_remain;
} aerial_robot_energy_t;

typedef __packed struct
{
		uint8_t armor_id : 4;
		uint8_t HP_deduction_reason : 4;
} ext_robot_hurt_t;

typedef __packed struct
{
		uint8_t bullet_type;
		uint8_t shooter_number;
		uint8_t launching_frequency;
		float initial_speed;
} ext_shoot_data_t;

typedef __packed struct
{
		uint16_t projectile_allowance_17mm;
		uint16_t projectile_allowance_42mm;
		uint16_t remaining_gold_coin;
} ext_projectile_allowance_t;

typedef __packed struct
{
		uint32_t rfid_status;
} ext_rfid_status_t;

typedef __packed struct
{
		uint8_t dart_launch_opening_status;
		uint8_t reserved;
		uint16_t target_change_time;
		uint16_t latest_launch_cmd_time;
} ext_dart_client_cmd_t;

//typedef __packed struct
//{
// uint16_t data_cmd_id;
// uint16_t sender_ID;
// uint16_t receiver_ID;
//}ext_student_interactive_header_data_t;

typedef __packed struct
{
		float hero_x;
		float hero_y;
		float engineer_x;
		float engineer_y;
		float standard_3_x;
		float standard_3_y;
		float standard_4_x;
		float standard_4_y;
		float standard_5_x;
		float standard_5_y;
}ground_robot_position_t;

typedef __packed struct
{
		uint8_t mark_hero_progress;
		uint8_t mark_engineer_progress;
		uint8_t mark_standard_3_progress;
		uint8_t mark_standard_4_progress;
		uint8_t mark_standard_5_progress;
		uint8_t mark_sentry_progress;
}radar_mark_data_t;

typedef __packed struct
{
		uint32_t sentry_info;
} sentry_info_t;

typedef __packed struct
{
		uint8_t radar_info;
} radar_info_t;

typedef __packed struct 
{ 
	float target_position_x; 
	float target_position_y; 
	uint8_t cmd_keyboard; 
	uint8_t target_robot_id; 
	uint8_t cmd_source; 
}map_command_t; 

typedef __packed struct 
{ 
	uint8_t intention; 
	uint16_t start_position_x; 
	uint16_t start_position_y; 
	int8_t delta_x[49]; 
	int8_t delta_y[49]; 
	uint16_t sender_id;
}map_data_t;

typedef __packed struct 
{ 
	uint32_t sentry_cmd; 
} sentry_cmd_t; 

enum judge_robot_ID{
		hero_red       = 1,
		engineer_red   = 2,
		infantry3_red  = 3,
		infantry4_red  = 4,
		infantry5_red  = 5,
		plane_red      = 6,
	
		hero_blue      = 101,
		engineer_blue  = 102,
		infantry3_blue = 103,
		infantry4_blue = 104,
		infantry5_blue = 105,
		plane_blue     = 106,
};

typedef __packed struct JUDGE_MODULE_DATA
{
     FrameHeader header;
     ext_game_status_t status;
     ext_game_result_t result;
     ext_game_robot_HP_t robot_hp;
//     ext_ICRA_buff_debuff_zone_and_lurk_status_t zone;
     ext_event_data_t event_data_t;
     ext_supply_projectile_action_t supply_status;
     ext_referee_warning_t warning;
     dart_info_t dart_remaining_time_t;
     ext_game_robot_status_t robot_status;
     ext_power_heat_data_t power_heat;
     ext_game_robot_pos_t robot_pos;
     ext_buff_t buff;
     aerial_robot_energy_t aerial_energy;
     ext_robot_hurt_t robot_hurt;
     ext_shoot_data_t shoot_data;
     ext_projectile_allowance_t bullet_remain;
     ext_rfid_status_t rfid_status_t;
     ext_dart_client_cmd_t dart_client_cmd;
		 ground_robot_position_t ground_robot_position;
		 radar_mark_data_t radar_mark_data;
		 sentry_info_t sentry_information;
		 radar_info_t radar_information;
		 map_command_t map_command;
}JUDGE_MODULE_DATA;

typedef __packed struct SENTRY_DATA
{
	sentry_cmd_t sentry_cmd;
	map_data_t map_data;
}SENTRY_DATA;

typedef __packed struct Sentry_t
{
	//=======================================================各种上限状态(10HZ)===============================================================//
	uint8_t Myself_level;		//本机器人的等级
	uint8_t Myself_id;	//本机器人的ID
	
	uint16_t red_remain_HP;	//红方机器人剩余血量
	uint16_t blue_remain_HP;	//蓝方机器人剩余血量
	
	uint16_t Myself_max_HP;	//本机器人血量上限
	
	uint16_t Myself_17mm_speed_limit;	//本机器人17mm射速限制
	uint16_t Myself_17mm_cooling_limit;		//本机器人17mm热量上限
	uint16_t Myself_17mm_cooling_rate;	//本机器人17mm热量每秒冷却值
	
	uint16_t Myself_42mm_speed_limit;	//本机器人42mm射速限制
	uint16_t Myself_chassis_power_limit;	//底盘功率上限

	//=======================================================实时数据(50HZ)===============================================================//
	uint8_t shooter_ID; //发射机构ID
	uint16_t Myself_chassis_power_buffer;	//实时缓冲能量
	float Myself_chassis_power;		//实时底盘功率
	uint16_t Myself_17mm_heat_id1;		//实时枪管1热量
	uint16_t Myself_17mm_heat_id2;		//实时枪管2热量
	
	//实时
	uint8_t bullet_frequence;	//实时射频枪管（单位为HZ）
	float bullet_speed;	//实时射速枪管(单位m/s)
	
	uint8_t armor_id:4;	//受伤的装甲板编号（应该0是非装甲板受伤，1-4是装甲板伤害，需测试验证）
	uint8_t hurt_type:4;	//受伤类型
	
	uint32_t rfid_status;
	uint8_t rfid[4];
	
	//=======================================================比赛数据===============================================================//
	uint8_t Flag_progress;	//比赛进程
	uint16_t Time_remain;		//比赛剩余时间
	uint8_t Flag_judge;		//判断红蓝方
	uint8_t supply_robot_id; //补弹机器人ID
	uint8_t supply_projectile_num; //补弹数量
	
	uint16_t red_base_HP; //红方基地血量
	uint16_t red_outpost_HP; //红方前哨战血量
	uint16_t blue_base_HP; //蓝方基地血量
	uint16_t blue_outpost_HP; //蓝方前哨战血量
	uint16_t my_outpost_HP; //己方前哨站血量
	
	uint32_t event_data; //能量机关、高地占领状态
	uint16_t projectile_allowance_17mm; //17mm弹丸允许发弹量
	uint16_t remaining_gold_coin; //剩余金币数量
	
	float target_position_x; 
	float target_position_y; 
	uint8_t cmd_keyboard; 
}Sentry_t;

//**********************************************发给裁判系统**********************************************//
typedef struct
{
	uint8_t Robot_Color;		// 机器人颜色
	uint16_t Robot_ID;			// 本机器人ID
	uint16_t Cilent_ID;			// 本机器人对应的客户端ID
	uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

/* 帧头定义 */
typedef __packed struct xFrameHeader
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
} xFrameHeader;

/****************************机器人交互数据****************************/
/* 发送的内容数据段最大为 113 检测是否超出大小限制?实际上图形段不会超，数据段最多30个，也不会超*/
/* 交互数据头结构 */
typedef __packed struct ext_student_interactive_header_data_t
{
	uint16_t data_cmd_id; // 由于存在多个内容 ID，但整个cmd_id 上行频率最大为 10Hz，请合理安排带宽。注意交互部分的上行频率
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

typedef __packed struct UI_CharReFresh_t //要加__packed！！！！否则发送的决策数据存在补齐的空位，裁判系统端无法正确分析数据
{
   xFrameHeader FrameHeader;
   uint16_t CmdID;
   ext_student_interactive_header_data_t datahead;
   uint8_t String_Data[4];
   uint16_t frametail;
} UI_CharReFresh_t; // 打印字符串数据

/* 交互数据长度 */
typedef enum
{
	Interactive_Data_LEN_Head = 6, //交互数据帧头
} Interactive_Data_Length_e;

/* 交互数据ID */
typedef enum
{
	UI_Data_ID_DrawChar = 0x110,
} Interactive_Data_ID_e;

/* 通信协议长度 */
typedef enum
{
	LEN_HEADER = 5, // 帧头长
	LEN_CMDID = 2,	// 命令码长度
	LEN_TAIL = 2,	// 帧尾CRC16

	LEN_CRC8 = 4, // 帧头CRC8校验长度=帧头+数据长+包序号
} JudgeFrameLength_e;

extern void UICharRefresh(uint8_t *string_Data);
//*******************************************************************************************************//

extern JUDGE_MODULE_DATA Judge_Hero;

extern Sentry_t Sentry;

void JUDGE_Receive(volatile uint8_t *databuffer,uint8_t length);
void JudgeSend(uint32_t TXData,uint16_t datacmdid);

extern void RefereeSend(uint8_t *send, uint16_t tx_len);
extern uint8_t get_uart5_tx_dma_busy_flag(void);
extern void clear_uart5_tx_dma_busy_sign(void);

#endif
