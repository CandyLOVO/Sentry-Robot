#include "Exchange_task.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "string.h"
#include "judge.h"

uint8_t Trans[8] = {0}; //CAN1�Ϸ���Ϣ

void Exchange_task(void const * argument)
{
  for(;;)
  {
		Trans[0] = Sentry.Flag_progress; //��������,1Ϊ׼���׶Σ�2Ϊ�Լ죬3Ϊ����ʱ��4Ϊ��ս�׶Σ�5Ϊ��������(����ʱ)
		Trans[1] = Sentry.Flag_judge; //�ж��ҷ��Ǻ췽��������,1-Red,2-Blue
		Trans[2] = Sentry.armor_id; //���˵�װ�װ��ţ�Ӧ��0�Ƿ�װ�װ����ˣ�1-4��װ�װ��˺�)
		Trans[3] = Sentry.Time_remain & 0xff; //����ʣ��ʱ��
		Trans[4] = (Sentry.Time_remain >> 8) & 0xff;
		can_remote(Trans,0x54);
    osDelay(1);
		
		Trans[0] = Sentry.Myself_remain_HP & 0xff; //��������ʣ��Ѫ����10HZ)
		Trans[1] = (Sentry.Myself_remain_HP >> 8) & 0xff;
		Trans[2] = Sentry.Myself_17mm_speed_id1 & 0xff; //ʵʱǹ��1����
		Trans[3] = (Sentry.Myself_17mm_speed_id1 >> 8) & 0xff;
		Trans[4] = Sentry.Myself_17mm_speed_id2 & 0xff; //ʵʱǹ��2����
		Trans[5] = (Sentry.Myself_17mm_speed_id2 >> 8) & 0xff;
		can_remote(Trans,0x55);
		osDelay(1);
		
		Trans[0] = Sentry.Myself_17mm_speed_limit_id1 & 0xff; //��������17mm��������-1
		Trans[1] = (Sentry.Myself_17mm_speed_limit_id1 >> 8) & 0xff;
		Trans[2] = Sentry.Myself_17mm_cooling_limit_id1 & 0xff; //��������17mm��������-1
		Trans[3] = (Sentry.Myself_17mm_cooling_limit_id1 >> 8) & 0xff;
		Trans[4] = Sentry.Myself_17mm_cooling_rate_id1 & 0xff; //��������17mm����ÿ����ȴֵ-1
		Trans[5] = (Sentry.Myself_17mm_cooling_rate_id1 >> 8) & 0xff;
		can_remote(Trans,0x61);
		osDelay(1);
		
		Trans[0] = (int32_t)Sentry.bullet_speed & 0xff; //ʵʱ����(��λm/s)//////////////////////////////ʵʱ����\ʵʱ��Ƶ
		Trans[1] = ((int32_t)Sentry.bullet_speed >> 8) & 0xff;
		Trans[2] = ((int32_t)Sentry.bullet_speed >> 16) & 0xff;
		Trans[3] = ((int32_t)Sentry.bullet_speed >> 24) & 0xff;
		Trans[4] = Sentry.Myself_17mm_speed_id1 & 0xff; //ʵʱǹ��1����-1
		Trans[5] = (Sentry.Myself_17mm_speed_id1 >> 8) & 0xff;
		Trans[6] = Sentry.bullet_frequence; //ʵʱ��Ƶ����λΪHZ��
		can_remote(Trans,0x62);
		osDelay(1);
		
		Trans[0] = Sentry.Myself_17mm_speed_limit_id2 & 0xff; //��������17mm��������-2
		Trans[1] = (Sentry.Myself_17mm_speed_limit_id2 >> 8) & 0xff;
		Trans[2] = Sentry.Myself_17mm_cooling_limit_id2 & 0xff; //��������17mm��������-2
		Trans[3] = (Sentry.Myself_17mm_cooling_limit_id2 >> 8) & 0xff;
		Trans[4] = Sentry.Myself_17mm_cooling_rate_id2 & 0xff; //��������17mm����ÿ����ȴֵ-2
		Trans[5] = (Sentry.Myself_17mm_cooling_rate_id2 >> 8) & 0xff;
		can_remote(Trans,0x63);
		osDelay(1);
		
		Trans[0] = (int32_t)Sentry.bullet_speed & 0xff; //ʵʱ����(��λm/s)//////////////////////////////ʵʱ����\ʵʱ��Ƶ
		Trans[1] = ((int32_t)Sentry.bullet_speed >> 8) & 0xff;
		Trans[2] = ((int32_t)Sentry.bullet_speed >> 16) & 0xff;
		Trans[3] = ((int32_t)Sentry.bullet_speed >> 24) & 0xff;
		Trans[4] = Sentry.Myself_17mm_speed_id2 & 0xff; //ʵʱǹ��2����
		Trans[5] = (Sentry.Myself_17mm_speed_id2 >> 8) & 0xff;
		Trans[6] = Sentry.bullet_frequence; //ʵʱ��Ƶ����λΪHZ��
		can_remote(Trans,0x64);
		osDelay(1);
		
		osDelay(1);
  }
}
