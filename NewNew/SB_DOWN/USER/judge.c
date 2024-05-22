#include "judge.h"
#include "CRC.h"
#include "main.h"
#include "struct_typedef.h"
#include "string.h"
#include "cmsis_os.h"

#define REFEREE_SOF 0xA5 // ��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define ID_sentry_cmd 0x0120 // �ڱ���������

uint8_t UI_Seq; // ����ţ�������referee�ļ�ʹ��
uint8_t Judge_Seq;
extern UART_HandleTypeDef huart5; //����ϵͳ UART5

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
                        memcpy((void*)(&Judge_Hero.status), (const void*)(&databuffer[pos+7]), data_length); //�������͡���ǰ�����׶Ρ���ǰ�׶�ʣ��ʱ��
                        break;
										case 0x0002:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.result), (const void*)(&databuffer[pos+7]), data_length); //ʤ��
                        break;
                    case 0x0003:
                        data_length = 32;
                         memcpy((void*)(&Judge_Hero.robot_hp), (const void*)(&databuffer[pos+7]), data_length); //�����������˼�����Ѫ��
                         break;
                    case 0x0101:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.event_data_t), (const void*)(&databuffer[pos+7]), data_length); //�����������ء��ߵ�ռ��״̬
                        break;
                    case 0x0102:
                        data_length = 4;
                         memcpy((void*)(&Judge_Hero.supply_status), (const void*)(&databuffer[pos+7]), data_length); //����
                         break;
                    case 0x0104 :
                        data_length = 3;
                        memcpy((void*)(&Judge_Hero.warning), (const void*)(&databuffer[pos+7]), data_length); //�з�
                        break;
                    case 0x0105 :
                        data_length = 3;
                        memcpy((void*)(&Judge_Hero.dart_remaining_time_t), (const void*)(&databuffer[pos+7]), data_length); //����
                         break;
                    case 0x0201:
                         data_length = 13;
                         memcpy((void*)(&Judge_Hero.robot_status), (const void*)(&databuffer[pos+7]), data_length); //������ID���ȼ���Ѫ����Ѫ�����ޡ�������ȴ�ٶȡ��������ޡ��������ޡ���Դ����ģ���������
                        break;
                    case 0x0202:
                        data_length = 16;
                        memcpy((void*)(&Judge_Hero.power_heat), (const void*)(&databuffer[pos+7]), data_length); //��Դ����ģ���chassis�������ѹ�����������̹��ʡ��������������������ǹ������
                        break;
                    case 0x0203:
                        data_length = 12;
                         memcpy((void*)(&Judge_Hero.robot_pos), (const void*)(&databuffer[pos+7]), data_length); //��������λ�á�����ģ�鳯��
                         break;
                    case 0x0204:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.buff), (const void*)(&databuffer[pos+7]), data_length); //����������
                        break;
                    case 0x0205:
                        data_length = 2;
                        memcpy((void*)(&Judge_Hero.aerial_energy), (const void*)(&databuffer[pos+7]), data_length); //���л�����
                        break;
                    case 0x0206:
                        data_length =1;
                        memcpy((void*)(&Judge_Hero.robot_hurt), (const void*)(&databuffer[pos+7]), data_length); //��Ѫԭ��Ѫ���仯����
                        break;
                    case 0x0207:
                        data_length = 7;
                        memcpy((void*)(&Judge_Hero.shoot_data), (const void*)(&databuffer[pos+7]), data_length); //������������� HZ��������ٶ� m/s
                        break;
                    case 0x0208:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.bullet_remain), (const void*)(&databuffer[pos+7]), data_length); //����������ʣ��������
                        break;
                    case 0x0209:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.rfid_status_t), (const void*)(&databuffer[pos+7]), data_length); //�Ƿ��Ѽ�⵽�������RFID��
                        break;
                    case 0x020A:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.dart_client_cmd), (const void*)(&databuffer[pos+7]), data_length); //���ڷ���վ
                        break;
										 case 0x020B:
                        data_length = 40;
                        memcpy((void*)(&Judge_Hero.ground_robot_position), (const void*)(&databuffer[pos+7]), data_length); //����������λ��
                        break;
										 case 0x020C:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.radar_mark_data), (const void*)(&databuffer[pos+7]), data_length); //�Է������˱���ǽ���
                        break;
										 case 0x020D:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.sentry_information), (const void*)(&databuffer[pos+7]), data_length); //�ڱ��ɹ��һ����������һ��������������һ�Ѫ������
                        break;
										 case 0x020E:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.radar_information), (const void*)(&databuffer[pos+7]), data_length); //�״�
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
	Sentry.Myself_chassis_power_limit = Judge_Hero.robot_status.chassis_power_limit;
	Sentry.Myself_17mm_heat_id1 = Judge_Hero.power_heat.shooter_17mm_1_barrel_heat;	//ʵʱǹ��1����
	Sentry.Myself_17mm_heat_id2 = Judge_Hero.power_heat.shooter_17mm_2_barrel_heat;	//ʵʱǹ��2����
	

	Sentry.shooter_ID = Judge_Hero.shoot_data.shooter_number; //�������ID��1-17mmǹ��1��2-17mmǹ��2��3-24mmǹ��
	Sentry.bullet_frequence = Judge_Hero.shoot_data.launching_frequency;	//������Ƶ����λΪHZ��
	Sentry.bullet_speed = Judge_Hero.shoot_data.initial_speed;	//�����ʼ����(��λm/s)
	Sentry.projectile_allowance_17mm = Judge_Hero.bullet_remain.projectile_allowance_17mm; //17mm������������
	Sentry.remaining_gold_coin = Judge_Hero.bullet_remain.remaining_gold_coin; //ʣ��������
	
	Sentry.supply_robot_id = Judge_Hero.supply_status.supply_robot_id; //����������ID
	Sentry.supply_projectile_num = Judge_Hero.supply_status.supply_projectile_num; //��������
	
	Sentry.armor_id = Judge_Hero.robot_hurt.armor_id;		//���˵�װ�װ��ţ�Ӧ��0�Ƿ�װ�װ����ˣ�1-4��װ�װ��˺����������֤��
	Sentry.hurt_type = Judge_Hero.robot_hurt.HP_deduction_reason;	//Ѫ���仯����
	
	//��������
	Sentry.Flag_progress =  Judge_Hero.status.game_progress;	//��������,1Ϊ׼���׶Σ�2Ϊ�Լ죬3Ϊ����ʱ��4Ϊ��ս�׶Σ�5Ϊ��������(����ʱ)
	Sentry.Time_remain = Judge_Hero.status.stage_remain_time;		//����ʣ��ʱ��
	Sentry.event_data = Judge_Hero.event_data_t.event_data; //���������㡢�������ء��ߵ�ռ��״̬�����ڻ���Ŀ�ꡢ���������ռ�����
	
	//�ж��ҷ��Ǻ췽����������������������ڱ���
	if(Sentry.Myself_id == 7)//��ɫ��
	{
		Sentry.Flag_judge = 1; //�жϺ췽�ı�־λ
		Sentry.my_outpost_HP = Sentry.red_outpost_HP;
	}
	else if(Sentry.Myself_id == 107)
	{
		Sentry.Flag_judge = 2; //�ж������ı�־λ
		Sentry.my_outpost_HP = Sentry.blue_outpost_HP;
	}
	
	Sentry.red_remain_HP = Judge_Hero.robot_hp.red_7_robot_HP; //�����ڱ�Ѫ��
	Sentry.red_outpost_HP = Judge_Hero.robot_hp.red_outpost_HP; //����ǰ��սѪ��
	Sentry.red_base_HP = Judge_Hero.robot_hp.red_base_HP; //��������Ѫ��
	
	Sentry.blue_remain_HP = Judge_Hero.robot_hp.blue_7_robot_HP; //�����ڱ�Ѫ��
	Sentry.blue_outpost_HP = Judge_Hero.robot_hp.blue_outpost_HP; //����ǰ��սѪ��
	Sentry.blue_base_HP = Judge_Hero.robot_hp.blue_base_HP; //��������Ѫ��
	
	Sentry.rfid_status = Judge_Hero.rfid_status_t.rfid_status;
//	memcpy(&Sentry.rfid[0], &Sentry.rfid_status, 4);
}

/**
 * @brief ����ϵͳ���ݷ��ͺ���
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
//	HAL_UART_Transmit_DMA(&huart5, send, tx_len);
	HAL_UART_Transmit_DMA(&huart5, send, tx_len);
//	osDelay(115);
}

/************************************************UI�����ַ���ʹ������Ч��*********************************/
void JudgeSend(uint32_t TXData,uint16_t datacmdid)
{
	static UI_CharReFresh_t senddatatoJudge;

	uint8_t temp_datalength = Interactive_Data_LEN_Head + 32; // ���㽻�����ݳ���

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

	RefereeSend((uint8_t *)&senddatatoJudge, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // ����

	Judge_Seq++; // �����+1
}
