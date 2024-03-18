#include "Exchange_task.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "string.h"
#include "judge.h"

uint8_t Trans[8] = {0}; //CAN1上发信息

void Exchange_task(void const * argument)
{
  for(;;)
  {
		Trans[0] = Sentry.Flag_progress; //比赛进程,1为准备阶段，2为自检，3为倒计时，4为对战阶段，5为比赛结束(结算时)
		Trans[1] = Sentry.Flag_judge; //判断我方是红方还是蓝方,1-Red,2-Blue
		Trans[2] = Sentry.armor_id; //受伤的装甲板编号（应该0是非装甲板受伤，1-4是装甲板伤害)
		Trans[3] = Sentry.Time_remain & 0xff; //比赛剩余时间
		Trans[4] = (Sentry.Time_remain >> 8) & 0xff;
		can_remote(Trans,0x54);
    osDelay(1);
		
		Trans[0] = Sentry.Myself_remain_HP & 0xff; //本机器人剩余血量（10HZ)
		Trans[1] = (Sentry.Myself_remain_HP >> 8) & 0xff;
		Trans[2] = Sentry.Myself_17mm_speed_id1 & 0xff; //实时枪管1热量
		Trans[3] = (Sentry.Myself_17mm_speed_id1 >> 8) & 0xff;
		Trans[4] = Sentry.Myself_17mm_speed_id2 & 0xff; //实时枪管2热量
		Trans[5] = (Sentry.Myself_17mm_speed_id2 >> 8) & 0xff;
		can_remote(Trans,0x55);
		osDelay(1);
		
		Trans[0] = Sentry.Myself_17mm_speed_limit_id1 & 0xff; //本机器人17mm射速限制-1
		Trans[1] = (Sentry.Myself_17mm_speed_limit_id1 >> 8) & 0xff;
		Trans[2] = Sentry.Myself_17mm_cooling_limit_id1 & 0xff; //本机器人17mm热量上限-1
		Trans[3] = (Sentry.Myself_17mm_cooling_limit_id1 >> 8) & 0xff;
		Trans[4] = Sentry.Myself_17mm_cooling_rate_id1 & 0xff; //本机器人17mm热量每秒冷却值-1
		Trans[5] = (Sentry.Myself_17mm_cooling_rate_id1 >> 8) & 0xff;
		can_remote(Trans,0x61);
		osDelay(1);
		
		Trans[0] = (int32_t)Sentry.bullet_speed & 0xff; //实时射速(单位m/s)//////////////////////////////实时射速\实时射频
		Trans[1] = ((int32_t)Sentry.bullet_speed >> 8) & 0xff;
		Trans[2] = ((int32_t)Sentry.bullet_speed >> 16) & 0xff;
		Trans[3] = ((int32_t)Sentry.bullet_speed >> 24) & 0xff;
		Trans[4] = Sentry.Myself_17mm_speed_id1 & 0xff; //实时枪管1热量-1
		Trans[5] = (Sentry.Myself_17mm_speed_id1 >> 8) & 0xff;
		Trans[6] = Sentry.bullet_frequence; //实时射频（单位为HZ）
		can_remote(Trans,0x62);
		osDelay(1);
		
		Trans[0] = Sentry.Myself_17mm_speed_limit_id2 & 0xff; //本机器人17mm射速限制-2
		Trans[1] = (Sentry.Myself_17mm_speed_limit_id2 >> 8) & 0xff;
		Trans[2] = Sentry.Myself_17mm_cooling_limit_id2 & 0xff; //本机器人17mm热量上限-2
		Trans[3] = (Sentry.Myself_17mm_cooling_limit_id2 >> 8) & 0xff;
		Trans[4] = Sentry.Myself_17mm_cooling_rate_id2 & 0xff; //本机器人17mm热量每秒冷却值-2
		Trans[5] = (Sentry.Myself_17mm_cooling_rate_id2 >> 8) & 0xff;
		can_remote(Trans,0x63);
		osDelay(1);
		
		Trans[0] = (int32_t)Sentry.bullet_speed & 0xff; //实时射速(单位m/s)//////////////////////////////实时射速\实时射频
		Trans[1] = ((int32_t)Sentry.bullet_speed >> 8) & 0xff;
		Trans[2] = ((int32_t)Sentry.bullet_speed >> 16) & 0xff;
		Trans[3] = ((int32_t)Sentry.bullet_speed >> 24) & 0xff;
		Trans[4] = Sentry.Myself_17mm_speed_id2 & 0xff; //实时枪管2热量
		Trans[5] = (Sentry.Myself_17mm_speed_id2 >> 8) & 0xff;
		Trans[6] = Sentry.bullet_frequence; //实时射频（单位为HZ）
		can_remote(Trans,0x64);
		osDelay(1);
		
		osDelay(1);
  }
}
