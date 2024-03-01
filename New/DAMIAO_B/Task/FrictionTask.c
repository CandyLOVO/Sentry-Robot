#include "FrictionTask.h"
#include "testTask.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "user_pid.h"
#include "rc_potocal.h"

#define mocalun_speed 15*15

pidTypeDef motor_m3508_pid[6];
pidTypeDef motor_m2006_pid[8];

//PID初始化
static void Friction_init(void);

//摩擦轮加速赋值
static void Friction_calc(void);

//摩擦轮减速赋值 
static void Friction_down(void);

//拨盘堵转检测
static void Bopan_judge(void);

//摩擦轮Pid输出值发送
extern void can_send_mocalun(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
//拨盘Pid输出值和发送
extern void can_send_bopan(int16_t motor1,int16_t motor2);


//拨盘PId计算
static void Bopan_calc(void);

//热量控制
static void Bopan_speed_calc_L(int speed);
static void Bopan_speed_calc_R(int speed);

//===============================================全局变量================================================//
extern ROBOT Sentry;
extern RC_ctrl_t rc_ctrl;
int16_t bopan_shoot_speed = 19*30;//90*36;	//拨盘发射弹丸转速
int16_t bopan_reversal_speed = -19*15;	//拨盘反转转速
uint8_t bopan_reversal_flag_L= 0,bopan_reversal_flag_R= 0;	//拨盘反转标志位，0为正转，1为反转

void FrictionTask(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	
	Friction_init();	//PID初始化
	
  for(;;)
  {
		//===============================================摩擦轮================================================//
		//开启摩擦轮
		if(1)//摩擦轮开启条件
		{
		Friction_calc();	//转速->电流
		}
		else//摩擦轮关闭
		{
			Friction_down();	
		}
		can_send_mocalun(motor_m3508[1].send_I,motor_m3508[2].send_I,motor_m3508[3].send_I,motor_m3508[4].send_I);//摩擦轮电流发送
		osDelay(1);
		//===============================================拨盘================================================//

		
		
		if(Sentry.Fire_flag_L==1)//左枪管发射
	{  if(bopan_reversal_flag_L==1)
				{
					Bopan_speed_calc_L(bopan_reversal_speed);
				}
			else if(bopan_reversal_flag_L==0)
			{
			Bopan_speed_calc_L(bopan_shoot_speed);		
	   	}
	}
				
	else{
	  Bopan_speed_calc_L(0);
	}
	
	
	
		if(Sentry.Fire_flag_R==1)//右枪管发射
	{  if(bopan_reversal_flag_R==1)
				{
					Bopan_speed_calc_R(bopan_reversal_speed);
				}
			else if(bopan_reversal_flag_R==0)
			{ 
			Bopan_speed_calc_R(bopan_shoot_speed);		
	   	}
	}
		else
		{		
			Bopan_speed_calc_R(0);	
		}

		
		//拨盘电流发射
		Bopan_calc();//转速-->电流
		Bopan_judge();//拨盘堵转检测
		can_send_bopan(motor_m2006[5].send_I,motor_m2006[6].send_I);
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

//===============================================PID初始化================================================//
static void Friction_init()
{
	pid_init(&motor_m3508_pid[1],15,0.8,1);//摩擦轮
	pid_init(&motor_m3508_pid[2],15,0.8,1);
	pid_init(&motor_m3508_pid[3],15,0.8,1);
	pid_init(&motor_m3508_pid[4],15,0.8,1);
	
	pid_init(&motor_m2006_pid[5],15,0.8,1);//拨盘(拨爪)
	pid_init(&motor_m2006_pid[6],15,0.8,1);//20，0.03，0.5
}

//==============================================摩擦轮转速->电流================================================//
static void Friction_calc()
{
	motor_m3508[1].set_v=mocalun_speed;
	motor_m3508[2].set_v=-mocalun_speed;
	motor_m3508[3].set_v=mocalun_speed;
	motor_m3508[4].set_v=-mocalun_speed;
	
	
	
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[1], motor_m3508[1].speed, motor_m3508[1].set_v,5000,2500);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[2], motor_m3508[2].speed, motor_m3508[2].set_v,5000,2500);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[3], motor_m3508[3].speed, motor_m3508[3].set_v,5000,2500);
	motor_m3508[4].send_I = pid_cal_s(&motor_m3508_pid[4], motor_m3508[4].speed, motor_m3508[4].set_v,5000,2500);
}

//===============================================摩擦轮减速到零================================================//
static void Friction_down()
{
	motor_m3508[1].set_v=0;
	motor_m3508[2].set_v=0;
	motor_m3508[3].set_v=0;
	motor_m3508[4].set_v=0;
	
	
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[1], motor_m3508[1].speed, motor_m3508[1].set_v,5000,2500);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[2], motor_m3508[2].speed, motor_m3508[2].set_v,5000,2500);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[3], motor_m3508[3].speed, motor_m3508[3].set_v,5000,2500);
	motor_m3508[4].send_I = pid_cal_s(&motor_m3508_pid[4], motor_m3508[4].speed, motor_m3508[4].set_v,5000,2500);
}



//===============================================拨盘PID计算================================================//
static void Bopan_calc()
{motor_m2006[5].send_I = pid_cal_s(&motor_m2006_pid[5], motor_m2006[5].speed, motor_m2006[5].set_v,5000,2500);
	motor_m2006[6].send_I = pid_cal_s(&motor_m2006_pid[6], motor_m2006[6].speed, motor_m2006[6].set_v,5000,2500);

}
//==========================================波盘速度计算（热量控制）======================================//

static void Bopan_speed_calc_L(int speed)
{if (Sentry.Cooling_heat_L<=350)
	{motor_m2006[5].set_v=speed;
	}
else if(Sentry.Cooling_heat_L>350)
{motor_m2006[5].set_v=389.5-0.95*Sentry.Cooling_heat_L;
}
}

static void Bopan_speed_calc_R(int speed)
{if (Sentry.Cooling_heat_R<=350)
	{motor_m2006[6].set_v=speed;
	}
else if(Sentry.Cooling_heat_R>350)
{motor_m2006[6].set_v=389.5-0.95*Sentry.Cooling_heat_R;
}
}

//拨盘堵转检测
static void Bopan_judge()
{if(motor_m2006[5].tor_current>5000)//修改堵转电流
	{bopan_reversal_flag_L=1;
	}
	else if(0>motor_m2006[5].tor_current && motor_m2006[5].tor_current>-100)
	{bopan_reversal_flag_L=0;
	
	}		
	
	if(motor_m2006[6].tor_current>5000)
	{bopan_reversal_flag_R=1;
	}
	else if(0>motor_m2006[6].tor_current && motor_m2006[6].tor_current>-50)
	{bopan_reversal_flag_R=0;
	}		
	
}
