#include "Launch_task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "pid_user.h"
#include "imu_temp_ctrl.h"
#include "Exchange_task.h"

pidTypeDef pid_mocalun_s[4];
pidTypeDef pid_bopan_s[2];
float target_speed = 19*380;
float target_mocalun[4];
float target_bopan[2];
int16_t mocalun_output[4];
int16_t bopan_output[2];
uint8_t mocalun_control[8];
uint8_t bopan_control[8];
uint16_t launch_heat_id1;
uint16_t launch_heat_id2;

extern FDCAN_HandleTypeDef hfdcan2;
extern motor_info motor_friction[8];
extern RC_ctrl_t rc_ctrl;
extern int8_t flag;
extern receive_vision Rx_vision;

//ºÚÍ·£¨×ó£©Ä¦²ÁÂÖID£º1¡¢2£¬²¦ÅÌID£º6£»°×Í·£¨ÓÒ£©Ä¦²ÁÂÖ£º3¡¢4£¬²¦ÅÌ£º5
void Launch_Task(void * argument)
{
	friction_init();
	
  for(;;)
  {
		//ÅÐ¶ÏÍÓÂÝÒÇÊÇ·ñÎÂ²¹½áÊø
		if(flag == 1)
		{
		if(rc_ctrl.rc.s[1]==1 && rc_ctrl.rc.s[0]==1)
		{
			mocalun_start();
//			target_bopan[0] = rc_ctrl.rc.ch[3] * (8*60/8*36)/660; //ºÚÍ·£¨×ó£©²¦ÅÌ n*60/8*36£ºn·¢/Ãë
//			target_bopan[1] = rc_ctrl.rc.ch[1] * (8*60/8*36)/660; //°×Í·£¨ÓÒ£©²¦ÅÌ
			target_bopan[0] = rc_ctrl.rc.ch[3] * (30*36)/660; //ºÚÍ·£¨×ó£©²¦ÅÌ n*60/8*36£ºn·¢/Ãë
			target_bopan[1] = rc_ctrl.rc.ch[1] * (30*36)/660; //°×Í·£¨ÓÒ£©²¦ÅÌ
		}
		
		else if(rc_ctrl.rc.s[1]==2 && rc_ctrl.rc.s[0]==2)
		{
			mocalun_start();
			if(Rx_vision.M_tracking == 1)
			{
				target_bopan[0] = 20/8*36;
				target_bopan[1] = 20/8*36;
			}
			if(Rx_vision.L_tracking == 1)
			{
				target_bopan[0] = 20/8*36;
			}
			if(Rx_vision.R_tracking == 1)
			{
				target_bopan[1] = 20/8*36;
			}
			else
			{
				target_bopan[0] = 0;
				target_bopan[1] = 0;
			}
//			if(launch_heat_id1 > 380) //ÈÈÁ¿ÏÞÖÆ
//			{
//				target_bopan[0] = 0;
//			}
//			else
//			{
////				target_bopan[0] = 3*60/8*36;
//				target_bopan[0] = 20*36;
//			}
//			
//			if(launch_heat_id2 > 380) //ÈÈÁ¿ÏÞÖÆ
//			{
//				target_bopan[1] = 0;
//			}
//			else
//			{
////				target_bopan[1] = 3*60/8*36;
//				target_bopan[1] = 20*36;
//			}
		}
		
		else
		{
			mocalun_stop();
			target_bopan[0] = 0;
			target_bopan[1] = 0;
		}
		
		mocalun_output[0] = pid_cal_s(&pid_mocalun_s[0], motor_friction[0].speed, target_mocalun[0]);
		mocalun_output[1] = pid_cal_s(&pid_mocalun_s[1], motor_friction[1].speed, target_mocalun[1]);
		mocalun_output[2] = pid_cal_s(&pid_mocalun_s[2], motor_friction[2].speed, target_mocalun[2]);
		mocalun_output[3] = pid_cal_s(&pid_mocalun_s[3], motor_friction[3].speed, target_mocalun[3]);
		bopan_output[0] = pid_cal_s(&pid_bopan_s[0], motor_friction[5].speed, -target_bopan[0]);
		bopan_output[1] = pid_cal_s(&pid_bopan_s[1], motor_friction[4].speed, -target_bopan[1]);
		
		mocalun_control[0] = (mocalun_output[0]>>8) & 0xff;
		mocalun_control[1] = mocalun_output[0] & 0xff;
		mocalun_control[2] = (mocalun_output[1]>>8) & 0xff;
		mocalun_control[3] = mocalun_output[1] & 0xff;
		mocalun_control[4] = (mocalun_output[2]>>8) & 0xff;
		mocalun_control[5] = mocalun_output[2] & 0xff;
		mocalun_control[6] = (mocalun_output[3]>>8) & 0xff;
		mocalun_control[7] = mocalun_output[3] & 0xff;
		canx_send_data(&hfdcan2, 0x200, mocalun_control, 8);
		osDelay(1);
		bopan_control[0] = (bopan_output[1]>>8) & 0xff;
		bopan_control[1] = bopan_output[1] & 0xff;
		bopan_control[2] = (bopan_output[0]>>8) & 0xff;
		bopan_control[3] = bopan_output[0] & 0xff;
		canx_send_data(&hfdcan2, 0x1ff, bopan_control, 8);
    osDelay(1);
		}
	}
}

void friction_init(void)
{
	pid_init(&pid_mocalun_s[0], 40, 0.8, 1, 16384, 16384); //ºÚÍ·£¨×ó£©Ä¦²ÁÂÖ 1¡¢2
	pid_init(&pid_mocalun_s[1], 40, 0.8, 1, 16384, 16384);
	pid_init(&pid_mocalun_s[2], 40, 0.8, 1, 16384, 16384); //°×Í·£¨ÓÒ£©Ä¦²ÁÂÖ 3¡¢4
	pid_init(&pid_mocalun_s[3], 40, 0.8, 1, 16384, 16384);
	
	pid_init(&pid_bopan_s[0], 20, 0.03, 0.5, 16384, 16384); //ºÚÍ·£¨×ó£©²¦ÅÌ 6
	pid_init(&pid_bopan_s[1], 20, 0.03, 0.5, 16384, 16384); //°×Í·£¨ÓÒ£©²¦ÅÌ 5
}

void mocalun_start(void)
{
	if(target_mocalun[0] > -target_speed)
	{
		target_mocalun[0] -= 10;
	}
	else
	{
		target_mocalun[0] = -target_speed;
	}
	if(target_mocalun[1] < target_speed)
	{
		target_mocalun[1] += 10;
	}
	else
	{
		target_mocalun[1] = target_speed;
	}
	if(target_mocalun[2] > -target_speed)
	{
		target_mocalun[2] -= 10;
	}
	else
	{
		target_mocalun[2] = -target_speed;
	}
	if(target_mocalun[3] < target_speed)
	{
		target_mocalun[3] += 10;
	}
	else
	{
		target_mocalun[3] = target_speed;
	}
}

void mocalun_stop(void)
{
//	mocalun_output[0] = 0;
//	mocalun_output[1] = 0;
//	mocalun_output[2] = 0;
//	mocalun_output[3] = 0;
	target_mocalun[0] = 0;
	target_mocalun[1] = 0;
	target_mocalun[2] = 0;
	target_mocalun[3] = 0;
}
