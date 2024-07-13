#ifndef JUDGE_H
#define JUDGE_H

#include "uart_user.h"


#define Cmdid_Judge_send 0x0301  //����ϵͳ����

#define Datacmd_Decision 0x0120  //�ڱ���������ָ��
#define ID_Dataccenter	 0x8080 //����ϵͳ������id�������ڱ����״���������ָ�
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
	//=======================================================��������״̬(10HZ)===============================================================//
	uint8_t Myself_level;		//�������˵ĵȼ�
	uint8_t Myself_id;	//�������˵�ID
	
	uint16_t red_remain_HP;	//�췽������ʣ��Ѫ��
	uint16_t blue_remain_HP;	//����������ʣ��Ѫ��
	
	uint16_t Myself_max_HP;	//��������Ѫ������
	
	uint16_t Myself_17mm_speed_limit;	//��������17mm��������
	uint16_t Myself_17mm_cooling_limit;		//��������17mm��������
	uint16_t Myself_17mm_cooling_rate;	//��������17mm����ÿ����ȴֵ
	
	uint16_t Myself_42mm_speed_limit;	//��������42mm��������
	uint16_t Myself_chassis_power_limit;	//���̹�������

	//=======================================================ʵʱ����(50HZ)===============================================================//
	uint8_t shooter_ID; //�������ID
	uint16_t Myself_chassis_power_buffer;	//ʵʱ��������
	float Myself_chassis_power;		//ʵʱ���̹���
	uint16_t Myself_17mm_heat_id1;		//ʵʱǹ��1����
	uint16_t Myself_17mm_heat_id2;		//ʵʱǹ��2����
	
	//ʵʱ
	uint8_t bullet_frequence;	//ʵʱ��Ƶǹ�ܣ���λΪHZ��
	float bullet_speed;	//ʵʱ����ǹ��(��λm/s)
	
	uint8_t armor_id:4;	//���˵�װ�װ��ţ�Ӧ��0�Ƿ�װ�װ����ˣ�1-4��װ�װ��˺����������֤��
	uint8_t hurt_type:4;	//��������
	
	uint32_t rfid_status;
	uint8_t rfid[4];
	
	//=======================================================��������===============================================================//
	uint8_t Flag_progress;	//��������
	uint16_t Time_remain;		//����ʣ��ʱ��
	uint8_t Flag_judge;		//�жϺ�����
	uint8_t supply_robot_id; //����������ID
	uint8_t supply_projectile_num; //��������
	
	uint16_t red_base_HP; //�췽����Ѫ��
	uint16_t red_outpost_HP; //�췽ǰ��սѪ��
	uint16_t blue_base_HP; //��������Ѫ��
	uint16_t blue_outpost_HP; //����ǰ��սѪ��
	uint16_t my_outpost_HP; //����ǰ��վѪ��
	
	uint32_t event_data; //�������ء��ߵ�ռ��״̬
	uint16_t projectile_allowance_17mm; //17mm������������
	uint16_t remaining_gold_coin; //ʣ��������
	
	float target_position_x; 
	float target_position_y; 
	uint8_t cmd_keyboard; 
}Sentry_t;

//**********************************************��������ϵͳ**********************************************//
typedef struct
{
	uint8_t Robot_Color;		// ��������ɫ
	uint16_t Robot_ID;			// ��������ID
	uint16_t Cilent_ID;			// �������˶�Ӧ�Ŀͻ���ID
	uint16_t Receiver_Robot_ID; // �����˳���ͨ��ʱ�����ߵ�ID������ͱ�������ͬ��ɫ
} referee_id_t;

/* ֡ͷ���� */
typedef __packed struct xFrameHeader
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
} xFrameHeader;

/****************************�����˽�������****************************/
/* ���͵��������ݶ����Ϊ 113 ����Ƿ񳬳���С����?ʵ����ͼ�ζβ��ᳬ�����ݶ����30����Ҳ���ᳬ*/
/* ��������ͷ�ṹ */
typedef __packed struct ext_student_interactive_header_data_t
{
	uint16_t data_cmd_id; // ���ڴ��ڶ������ ID��������cmd_id ����Ƶ�����Ϊ 10Hz��������Ŵ���ע�⽻�����ֵ�����Ƶ��
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

typedef __packed struct UI_CharReFresh_t //Ҫ��__packed�������������͵ľ������ݴ��ڲ���Ŀ�λ������ϵͳ���޷���ȷ��������
{
   xFrameHeader FrameHeader;
   uint16_t CmdID;
   ext_student_interactive_header_data_t datahead;
   uint8_t String_Data[4];
   uint16_t frametail;
} UI_CharReFresh_t; // ��ӡ�ַ�������

/* �������ݳ��� */
typedef enum
{
	Interactive_Data_LEN_Head = 6, //��������֡ͷ
} Interactive_Data_Length_e;

/* ��������ID */
typedef enum
{
	UI_Data_ID_DrawChar = 0x110,
} Interactive_Data_ID_e;

/* ͨ��Э�鳤�� */
typedef enum
{
	LEN_HEADER = 5, // ֡ͷ��
	LEN_CMDID = 2,	// �����볤��
	LEN_TAIL = 2,	// ֡βCRC16

	LEN_CRC8 = 4, // ֡ͷCRC8У�鳤��=֡ͷ+���ݳ�+�����
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
