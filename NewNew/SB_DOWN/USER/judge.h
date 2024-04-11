#ifndef JUDGE_H
#define JUDGE_H

#include "uart_user.h"
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

typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3;
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3;
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3;
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3;
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3;
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
uint16_t blue1_bullet_left;
uint16_t blue2_bullet_left;
uint8_t lurk_mode;
uint8_t res;
} ext_ICRA_buff_debuff_zone_and_lurk_status_t;



typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;



typedef __packed struct
{
    uint8_t supply_projectile_id;
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
} ext_dart_remaining_time_t;


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
}ext_buff_t;


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
	int8_t bullet_type;
  uint8_t shooter_number;
  uint8_t launching_frequency;
  float initial_speed;
} ext_shoot_data_t;

typedef __packed struct
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
} ext_bullet_remaining_t;


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

typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t sender_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

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
     ext_ICRA_buff_debuff_zone_and_lurk_status_t zone;
     ext_event_data_t event_data;
     ext_supply_projectile_action_t supply_status;
     ext_referee_warning_t warning;
     ext_dart_remaining_time_t dart_remaining_time_t;


     ext_game_robot_status_t    robot_status;
     ext_power_heat_data_t power_heat;
     ext_game_robot_pos_t robot_pos;
     ext_buff_t buff;

     aerial_robot_energy_t aerial_energy;

     ext_robot_hurt_t robot_hurt;
     ext_shoot_data_t shoot_data;
     ext_bullet_remaining_t bullet_remain;

     ext_rfid_status_t rfid_status_t;
     ext_dart_client_cmd_t dart_client_cmd;
		 
		 ground_robot_position_t ground_robot_position;
		 radar_mark_data_t radar_mark_data;
		 sentry_info_t sentry_information;
		 radar_info_t radar_information;
}JUDGE_MODULE_DATA;

typedef __packed struct Sentry_t
{
	//=======================================================各种上限状态(10HZ)===============================================================//
	uint8_t Myself_level;		//本机器人的等级
	uint8_t Myself_id;	//本机器人的ID
	uint16_t Myself_remain_HP;	//本机器人剩余血量
	uint16_t Myself_max_HP;	//本机器人血量上限
	//枪管1
	uint16_t Myself_17mm_speed_limit_id1;	//本机器人17mm射速限制
	uint16_t Myself_17mm_cooling_limit_id1;		//本机器人17mm热量上限
	uint16_t Myself_17mm_cooling_rate_id1;	//本机器人17mm热量每秒冷却值
	//枪管2
	uint16_t Myself_17mm_speed_limit_id2;	//本机器人17mm射速限制
	uint16_t Myself_17mm_cooling_limit_id2;		//本机器人17mm热量上限
	uint16_t Myself_17mm_cooling_rate_id2;	//本机器人17mm热量每秒冷却值
	
	uint16_t Myself_42mm_speed_limit;	//本机器人42mm射速限制
	uint16_t Myself_chassis_power_limit;	//底盘功率上限

	//=======================================================实时数据(50HZ)===============================================================//
	uint8_t shooter_ID; //发射机构ID
	uint16_t Myself_chassis_power_buffer;	//实时缓冲能量
	float Myself_chassis_power;		//实时底盘功率
	uint16_t Myself_17mm_speed_id1;		//实时枪管1热量
	uint16_t Myself_17mm_speed_id2;		//实时枪管2热量
	
	//实时
	uint8_t bullet_frequence_1;	//实时射频枪管1（单位为HZ）
	float bullet_speed_1;	//实时射速枪管1(单位m/s)
	uint8_t bullet_frequence_2;	//实时射频枪管2（单位为HZ）
	float bullet_speed_2;	//实时射速枪管2(单位m/s)
	uint8_t armor_id:4;	//受伤的装甲板编号（应该0是非装甲板受伤，1-4是装甲板伤害，需测试验证）
	uint8_t hurt_type:4;	//受伤类型
	
	//=======================================================比赛数据===============================================================//
	uint8_t Flag_progress;	//比赛进程
	uint16_t Time_remain;		//比赛剩余时间
	uint8_t Flag_judge;		//判断红蓝方
	uint8_t Flag_first;		//哨兵开环方案专用
	uint8_t supply_projectile_num;
	uint16_t base_HP;
	
}Sentry_t;

extern JUDGE_MODULE_DATA Judge_Hero;

extern Sentry_t Sentry;

void JUDGE_Receive(volatile uint8_t *databuffer,uint8_t length);

#endif
