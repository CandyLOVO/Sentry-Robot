#include "FrictionTask.h"
#include "testTask.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "user_pid.h"
#include "rc_potocal.h"


#define mocalun_speed   19*350    //摩擦轮转速(根据实际情况更改快速调整射速）


#define K_shoot_rate_correct 1 //射频修正参数（根据实际情况更改快速调整射频）
#define C_bopan_block_I 6000   //拨盘堵转电流（测试后更改）
#define C_bopan_unblock_I 50   //拨盘正常旋转电流（测试后更改）
#define K_rc_to_bopanSpeed 3 //遥控通道值切换到拨盘速度（更改可快速调整1-1模式下遥控与拨盘映射关系）


pidTypeDef motor_m3508_pid[4];
pidTypeDef motor_m2006_pid[2];



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
static void Bopan_speed_calc_L(int speed_rate_high, int speed_rate_low,int speed_rate_test);
static void Bopan_speed_calc_R(int speed_rate_high, int speed_rate_low,int speed_rate_test);

//遥控控制
static void re_control(void);
static void re_shoot_rate_calc(void);

//===============================================全局变量================================================//
extern Shooter_t Shooter_L;
extern Shooter_t Shooter_R;
extern RC_ctrl_t rc_ctrl;
extern RC_ctrl_t rc_ctrl;
int16_t bopan_shoot_rate_max = 360;	//最高射频（个/min）
int16_t bopan_shoot_rate_min = 240; //最低射频
int16_t bopan_shoot_rate_test = 100;//无裁判系统射频
int16_t bopan_reversal_shoot_rate = -100;	//拨盘反转射频
uint8_t bopan_reversal_flag_L= 0,bopan_reversal_flag_R= 0;	//拨盘反转标志位，0为正转，1为反转
int16_t remote_mode;         //遥控模式，1-1为Fire Control 模式，摩擦轮旋转，遥控左右摇杆控制拨盘旋转；2-2为上场模式，摩擦轮旋转，根据视觉识别位发弹
int bopan_back_targetPos;    //拨盘反转位置环，目标位置


void FrictionTask(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	
	Friction_init();	//PID初始化
	
  for(;;)
  {re_control();
		re_shoot_rate_calc();
		//===============================================摩擦轮================================================//
		//开启摩擦轮
		if(remote_mode==22 || remote_mode==11)//摩擦轮开启条件
		{
		Friction_calc();	//转速->电流
		}
		else//摩擦轮关闭
		{
			Friction_down();	
		}
		can_send_mocalun(motor_m3508[0].send_I,motor_m3508[1].send_I,motor_m3508[2].send_I,motor_m3508[3].send_I);//摩擦轮电流发送
		osDelay(1);
		//===============================================拨盘================================================//
      //===========================================自动模式 begin================================//
if(remote_mode==22)
{
		
		if(Shooter_L.Fire_Flag==1 )//左枪管发射
	{  if(bopan_reversal_flag_L==1)
			{
			Bopan_speed_calc_L(bopan_reversal_shoot_rate,bopan_reversal_shoot_rate,bopan_reversal_shoot_rate);//最高射频，最低射频，无裁判系统射频
			}
			else if(bopan_reversal_flag_L==0)
			{
			Bopan_speed_calc_L(bopan_shoot_rate_max,bopan_shoot_rate_min,bopan_shoot_rate_test);		
	   	}
	}
				
	  else
	{
	  Bopan_speed_calc_L(0,0,0);
	}
	
	
	
		if(Shooter_R.Fire_Flag==1)//右枪管发射
	{  if(bopan_reversal_flag_R==1)
		  {
			Bopan_speed_calc_R(bopan_reversal_shoot_rate,bopan_reversal_shoot_rate,bopan_reversal_shoot_rate);//最高射频，最低射频，无裁判系统射频
			}
		 else if(bopan_reversal_flag_R==0)
			{
			Bopan_speed_calc_R(bopan_shoot_rate_max,bopan_shoot_rate_min,bopan_shoot_rate_test);		
	   	}
	}
				
	  else
  {
	  Bopan_speed_calc_R(0,0,0);
	}
	
}
//===========================自动模式 end==============================//  
		
else if (remote_mode==11)
{
		re_shoot_rate_calc();  //遥控控制拨盘，左右摇杆上推

}

else
{
Bopan_speed_calc_L(0,0,0);
Bopan_speed_calc_R(0,0,0);
}
		            
		Bopan_calc();//转速-->电流
		Bopan_judge();//拨盘堵转检测
		can_send_bopan(motor_m2006[0].send_I,motor_m2006[1].send_I);//拨盘电流发送
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

//===================================================遥控控制===============================================//
static void re_control()
{
		if(rc_ctrl.rc.s[1]==2 && rc_ctrl.rc.s[0]==2)
		{
		remote_mode=22;
		}
    else if (rc_ctrl.rc.s[1]==1 &&rc_ctrl.rc.s[0]==1)
		{
		remote_mode=11;
		}
		else
		{
		remote_mode=0;
		}
}

static void re_shoot_rate_calc()
{
	motor_m2006[0].set_v=rc_ctrl.rc.ch[3]*K_rc_to_bopanSpeed;
	motor_m2006[1].set_v=rc_ctrl.rc.ch[1]*K_rc_to_bopanSpeed;

}

//===============================================PID初始化================================================//
static void Friction_init()
{
	pid_init(&motor_m3508_pid[0],40,0.8,1);//摩擦轮
	pid_init(&motor_m3508_pid[1],40,0.8,1);
	pid_init(&motor_m3508_pid[2],40,0.8,1);
	pid_init(&motor_m3508_pid[3],40,0.8,1);
	
	pid_init(&motor_m2006_pid[0],15,0.03,0.5);//拨盘()
	pid_init(&motor_m2006_pid[1],15,0.03,0.5);//20，0.03，0.5
	
	Shooter_L.shooter_heat=1025;
	Shooter_R.shooter_heat=1025;
}

//==============================================摩擦轮转速->电流================================================//
static void Friction_calc()
{
	motor_m3508[0].set_v=-mocalun_speed;
	motor_m3508[1].set_v= mocalun_speed;
	motor_m3508[2].set_v=-mocalun_speed;
	motor_m3508[3].set_v= mocalun_speed;
	
	motor_m3508[0].send_I = pid_cal_s(&motor_m3508_pid[0], motor_m3508[0].speed, motor_m3508[0].set_v,16384,16384);
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[1], motor_m3508[1].speed, motor_m3508[1].set_v,16384,16384);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[2], motor_m3508[2].speed, motor_m3508[2].set_v,16384,16384);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[3], motor_m3508[3].speed, motor_m3508[3].set_v,16384,16384);

}

//===============================================摩擦轮减速到零================================================//
static void Friction_down()
{
	motor_m3508[0].set_v=0;
	motor_m3508[1].set_v=0;
	motor_m3508[2].set_v=0;
	motor_m3508[3].set_v=0;
	
	
	motor_m3508[0].send_I = pid_cal_s(&motor_m3508_pid[0], motor_m3508[0].speed, motor_m3508[0].set_v,16384,16384);
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[1], motor_m3508[1].speed, motor_m3508[1].set_v,16384,16384);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[2], motor_m3508[2].speed, motor_m3508[2].set_v,16384,16384);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[3], motor_m3508[3].speed, motor_m3508[3].set_v,16384,16384);
}



//===============================================拨盘PID计算================================================//
static void Bopan_calc()
{
	
	motor_m2006[0].send_I = pid_cal_s(&motor_m2006_pid[0], motor_m2006[0].speed, -motor_m2006[0].set_v,16384,16384);
	motor_m2006[1].send_I = pid_cal_s(&motor_m2006_pid[1], motor_m2006[1].speed, -motor_m2006[1].set_v,16384,16384);
}
//==========================================波盘速度计算（热量控制）======================================//

static void Bopan_speed_calc_L(int speed_rate_high, int speed_rate_low,int speed_rate_test)//变量：最高射频，最低射频，无裁判系统射频
{
	if (Shooter_L.shooter_heat<=300)//热量小于300，以最高射频发弹
	{
		motor_m2006[0].set_v=speed_rate_high/8*36*K_shoot_rate_correct;//电机转速（rpm）=射频（个/min）/8(一圈拨盘8个弹）*36（电机拨盘减速比 36：1）
	}
	else if(Shooter_L.shooter_heat>300 && Shooter_L.shooter_heat<=360)//300<热量<=360，根据热量降射频
	{
		motor_m2006[0].set_v=((speed_rate_low-speed_rate_high)/60*Shooter_L.shooter_heat+6*speed_rate_high-5*speed_rate_low)/8*36*K_shoot_rate_correct;
}
else if(Shooter_L.shooter_heat>360 && Shooter_L.shooter_heat<400)//360<热量<400,以最低射频发弹
{
	motor_m2006[0].set_v=speed_rate_low/8*36*K_shoot_rate_correct;
}
else if(Shooter_L.shooter_heat==1025)   //无裁判系统时初始化枪管热量为1025，低速发弹
{
	motor_m2006[0].set_v=speed_rate_test/8*36*K_shoot_rate_correct;
}
else                                    //超热量或裁判系统故障，发弹暂停
{
	motor_m2006[0].set_v=0;
}
}

static void Bopan_speed_calc_R(int speed_rate_high, int speed_rate_low,int speed_rate_test)//变量：最高射频，最低射频，无裁判系统射频
{
	if (Shooter_R.shooter_heat<=300) //热量小于300，以最高射频发弹
	{
		motor_m2006[1].set_v=speed_rate_high/8*36*K_shoot_rate_correct;//电机转速（rpm）=射频（个/min）/8(一圈拨盘8个弹）*36（电机拨盘减速比 36：1）
	}
	else if(Shooter_R.shooter_heat>300 && Shooter_R.shooter_heat<=360)//300<热量<=360，根据热量降射频
	{
		motor_m2006[1].set_v=((speed_rate_low-speed_rate_high)/60*Shooter_R.shooter_heat+6*speed_rate_high-5*speed_rate_low)/8*36*K_shoot_rate_correct;
	}
	else if(Shooter_R.shooter_heat>360 && Shooter_R.shooter_heat<400)//360<热量<400,以最低射频发弹
	{
		motor_m2006[1].set_v=speed_rate_low/8*36;
	}
	else if(Shooter_R.shooter_heat==1025)   //无裁判系统时初始化枪管热量为1025，以测试射频发弹
	{
		motor_m2006[1].set_v=speed_rate_test/8*36*K_shoot_rate_correct;
	}
	else                                    //超热量或裁判系统故障，发弹暂停
	{
		motor_m2006[1].set_v=0;
	}
}
//=====================================================拨盘堵转检测=======================================//
static void Bopan_judge()
{if(motor_m2006[0].tor_current<-C_bopan_block_I)
	{
		bopan_reversal_flag_L=1;
	}
	else if(motor_m2006[0].tor_current>C_bopan_block_I)
	{
		bopan_reversal_flag_L=0;
	}		
	
	if(motor_m2006[1].tor_current<-C_bopan_block_I)
	{
		bopan_reversal_flag_R=1;
	}
	else if(motor_m2006[1].tor_current>C_bopan_block_I)
	{
		bopan_reversal_flag_R=0;
	}		
	
}
//=========================================拨盘退弹==============================================//
/*
static void Bopan_back_L(int flag)  //反转左拨盘
{
	bopan_back_targetPos=motor_m2006[0].angle+C_bopan_back_position;
	 if(bopan_back_targetPos>8191)
	 {
		 bopan_back_targetPos=bopan_back_targetPos-8191;
	}
	
	
	
}	*/
