#include "judge.h"
#include "CRC.h"
#include "main.h"
#include "struct_typedef.h"
#include "string.h"

//=======================================================����ϵͳУ���ȡ(���ж���ʽ)===============================================================//
extern uint8_t first_x;
extern uint8_t first_y;

//����2���ṹ�壬һ���ٷ����յģ�һ���Լ�ʹ�õ�
JUDGE_MODULE_DATA Judge_Hero;
Sentry_t Sentry;
int32_t event;
int8_t data[4];

static void Update_data();//���¶���һЩ��Ҫ�õ��ı�����ʵʱ������ֵ���������ļ����ã���Ҫ��������������޸ģ�
	
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
                         memcpy((void*)(&Judge_Hero.robot_status), (const void*)(&databuffer[pos+7]), data_length);   //���̹���������������
                        break;
                    case 0x0202:
                        data_length = 16;
                        memcpy((void*)(&Judge_Hero.power_heat), (const void*)(&databuffer[pos+7]), data_length);      //��ʵʱ������������
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
		 //У����ɺ�
		 Update_data();
}


static void Update_data()
{
//=======================================================��������״̬(10HZ)===============================================================//
	Sentry.Myself_id = Judge_Hero.robot_status.robot_id;	//�������˵�ID
	Sentry.Myself_level = Judge_Hero.robot_status.robot_level;		//�������˵ĵȼ�

	
//=======================================================ʵʱ����(50HZ)===============================================================//
	Sentry.Myself_chassis_power_buffer = Judge_Hero.power_heat.buffer_energy;	//ʵʱ��������
	Sentry.Myself_chassis_power = Judge_Hero.power_heat.chassis_power;	//ʵʱ���̹���
	
	Sentry.Myself_17mm_speed_id1 = Judge_Hero.power_heat.shooter_17mm_1_barrel_heat;	//ʵʱǹ��1����
	Sentry.Myself_17mm_speed_id2 = Judge_Hero.power_heat.shooter_17mm_2_barrel_heat;	//ʵʱǹ��2����
	

	Sentry.shooter_ID = Judge_Hero.shoot_data.shooter_number; //�������ID��1-17mmǹ��1��2-17mmǹ��2��3-24mmǹ��
	if(Sentry.shooter_ID==1)
	{
		Sentry.bullet_frequence_1 = Judge_Hero.shoot_data.launching_frequency;	//ʵʱ��Ƶ����λΪHZ��
		Sentry.bullet_speed_1 = Judge_Hero.shoot_data.initial_speed;	//ʵʱ����(��λm/s)
	}
	if(Sentry.shooter_ID==2)
	{
		Sentry.bullet_frequence_2 = Judge_Hero.shoot_data.launching_frequency;	//ʵʱ��Ƶ����λΪHZ��
	  Sentry.bullet_speed_2 = Judge_Hero.shoot_data.initial_speed;	//ʵʱ����(��λm/s)
	}
	
	Sentry.armor_id = Judge_Hero.robot_hurt.armor_id;		//���˵�װ�װ��ţ�Ӧ��0�Ƿ�װ�װ����ˣ�1-4��װ�װ��˺����������֤��
	Sentry.hurt_type = Judge_Hero.robot_hurt.HP_deduction_reason;	//Ѫ���仯����
	
	//��������
	Sentry.Flag_progress =  Judge_Hero.status.game_progress;	//��������,1Ϊ׼���׶Σ�2Ϊ�Լ죬3Ϊ����ʱ��4Ϊ��ս�׶Σ�5Ϊ��������(����ʱ)
	Sentry.Time_remain = Judge_Hero.status.stage_remain_time;		//����ʣ��ʱ��
	event = Judge_Hero.event_data.event_type;	
	memcpy(&data[0],&event,4);
	
	//�ж��ҷ��Ǻ췽����������������������ڱ���
	if(Sentry.Myself_id == 7)//��ɫ��
	{
		Sentry.Flag_judge = 1;
		Sentry.Myself_remain_HP = Judge_Hero.robot_hp.red_7_robot_HP;
		Sentry.base_HP = Judge_Hero.robot_hp.blue_base_HP; //�з�����Ѫ��
	}
	else if(Sentry.Myself_id == 107)
	{
		Sentry.Flag_judge = 2;
		Sentry.Myself_remain_HP = Judge_Hero.robot_hp.blue_7_robot_HP;
		Sentry.base_HP = Judge_Hero.robot_hp.red_base_HP; //�з�����Ѫ��
	}
	Sentry.supply_projectile_num = Judge_Hero.supply_status.supply_projectile_num;
}
