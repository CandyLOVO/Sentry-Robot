#include "FrictionTask_v2.h"
#include "testTask.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "user_pid.h"
#include "rc_potocal.h"

#define K_shoot_speed_correct 1 //射速修正参数（根据实际情况更改快速调整射速） 
#define K_shoot_rate_correct 1 //射频修正参数（根据实际情况更改快速调整射频）
#define C_bopan_block_I 5000   //拨盘堵转电流（测试后更改）
#define C_speed_protect 0.2    //射速冗余（m/s）
#define C_heat_protect 20      //热量冗余
#define C_bopan_unblock_I 50   //拨盘正常旋转电流（测试后更改）

extern pidTypeDef motor_m3508_pid[4]; 
extern pidTypeDef motor_m2006_pid[2];

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
static void Bopan_speed_calc_L(void);
static void Bopan_speed_calc_R(void);

//===============================================全局变量================================================//
extern Shooter_t Shooter_L;
extern Shooter_t Shooter_R;
extern RC_ctrl_t rc_ctrl;
int16_t bopan_shoot_speed ;	//拨盘电机发射转速
int16_t bopan_reversal_speed = -19*15;	//拨盘反转转速
extern uint8_t bopan_reversal_flag_L;
extern uint8_t bopan_reversal_flag_R;	//拨盘反转标志位，0为正转，1为反转
int I_test;
void FrictionTask_v2(void const * argument)
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
		can_send_mocalun(motor_m3508[0].send_I,motor_m3508[1].send_I,motor_m3508[2].send_I,motor_m3508[3].send_I);//摩擦轮电流发送
		osDelay(1);
		
		//===============================================拨盘================================================//		
		if(Shooter_L.Fire_Flag==1)//左枪管发射
	{  if(bopan_reversal_flag_L==1)
				{
					motor_m2006[0].set_v=bopan_reversal_speed;//2006目标速度
				}
			else if(bopan_reversal_flag_L==0)
			{
				
			Bopan_speed_calc_L();		
	   	}
	}
				
	else{
	  motor_m2006[0].set_v=0;
	}
	
	
	
	if(Shooter_R.Fire_Flag==1)//右枪管发射
	{  if(bopan_reversal_flag_R==1)
				{
					motor_m2006[1].set_v=bopan_reversal_speed;//2006目标速度
				}
			else if(bopan_reversal_flag_R==0)
			{
				
			Bopan_speed_calc_R();		
	   	}
	}
				
	else{
	  motor_m2006[1].set_v=0;
	}

		
		//拨盘电流发射
		Bopan_calc();//转速-->电流
		Bopan_judge();//拨盘堵转检测
		can_send_bopan(motor_m2006[0].send_I,motor_m2006[1].send_I);
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

//===============================================PID初始化================================================//
static void Friction_init()
{
	pid_init(&motor_m3508_pid[0],15,0.8,1);//摩擦轮
	pid_init(&motor_m3508_pid[1],15,0.8,1);
	pid_init(&motor_m3508_pid[2],15,0.8,1);
	pid_init(&motor_m3508_pid[3],15,0.8,1);
	
	pid_init(&motor_m2006_pid[0],15,0.8,1);//拨盘(拨爪)
	pid_init(&motor_m2006_pid[1],15,0.8,1);//20，0.03，0.5
	

}

//==============================================摩擦轮转速->电流================================================//
static void Friction_calc()
{
	int target_shoot_speed_L,target_shoot_speed_R;
	target_shoot_speed_L=(Shooter_L.speed_limit-C_speed_protect)*K_shoot_speed_correct;  //目标射速=（射速限制-射速冗余）*射速修正系数
	target_shoot_speed_R=(Shooter_R.speed_limit-C_speed_protect)*K_shoot_speed_correct;
	
	motor_m3508[0].set_v=target_shoot_speed_L;
	motor_m3508[1].set_v=-target_shoot_speed_L;
	motor_m3508[2].set_v=target_shoot_speed_R;
	motor_m3508[3].set_v=-target_shoot_speed_R;
	
/*	motor_m3508[0].send_I = pid_cal_s(&motor_m3508_pid[0], Shooter_L.shoot_speed, motor_m3508[0].set_v,5000,2500);
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[1], Shooter_L.shoot_speed, motor_m3508[1].set_v,5000,2500);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[2], Shooter_R.shoot_speed, motor_m3508[2].set_v,5000,2500);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[3], Shooter_R.shoot_speed, motor_m3508[3].set_v,5000,2500);*/
	motor_m3508[0].send_I = I_test;
	motor_m3508[1].send_I = I_test;
	motor_m3508[2].send_I = I_test;
	motor_m3508[3].send_I = I_test;
}

//===============================================摩擦轮减速到零================================================//
static void Friction_down()
{
	motor_m3508[0].set_v=0;
	motor_m3508[1].set_v=0;
	motor_m3508[2].set_v=0;
	motor_m3508[3].set_v=0;
	
	
	motor_m3508[0].send_I = pid_cal_s(&motor_m3508_pid[1], motor_m3508[1].speed, motor_m3508[1].set_v,5000,2500);
	motor_m3508[1].send_I = pid_cal_s(&motor_m3508_pid[2], motor_m3508[2].speed, motor_m3508[2].set_v,5000,2500);
	motor_m3508[2].send_I = pid_cal_s(&motor_m3508_pid[3], motor_m3508[3].speed, motor_m3508[3].set_v,5000,2500);
	motor_m3508[3].send_I = pid_cal_s(&motor_m3508_pid[4], motor_m3508[4].speed, motor_m3508[4].set_v,5000,2500);
}



//===============================================拨盘PID计算================================================//
static void Bopan_calc()
{ 
	motor_m2006[0].send_I = pid_cal_s(&motor_m2006_pid[0], Shooter_L.shoot_rate, motor_m2006[0].set_v,5000,2500);
	motor_m2006[1].send_I = pid_cal_s(&motor_m2006_pid[1], Shooter_R.shoot_rate, motor_m2006[1].set_v,5000,2500);
}
//==========================================波盘速度计算（热量控制）======================================//

static void Bopan_speed_calc_L()
{int target_shoot_rate_L;  //目标射频
	int heat_space;          //热量增长空间
	heat_space=Shooter_L.heat_limit-Shooter_L.shooter_heat; //热量增长空间=热量上限-实时热量
	if(heat_space>=100)
	{
	target_shoot_rate_L=Shooter_L.cooling_rate/10*2;
	}
	else if(100>heat_space && heat_space>=50)
	{
	target_shoot_rate_L=Shooter_L.cooling_rate/10;
	
	}
	else if(50>heat_space && heat_space>=C_heat_protect)
 {
	target_shoot_rate_L=Shooter_L.cooling_rate/10*0.8;
	}
 else
 {
 target_shoot_rate_L=0;
 }
	
motor_m2006[0].set_v=target_shoot_rate_L*K_shoot_rate_correct;
	
}


static void Bopan_speed_calc_R()
{int target_shoot_rate_R;  //目标射频
	int heat_space;          //热量增长空间
	heat_space=Shooter_R.heat_limit-Shooter_R.shooter_heat; //热量增长空间=热量上限-实时热量
	if(heat_space>100)
	{
	target_shoot_rate_R=Shooter_R.cooling_rate/10*2;
	}
	else if(100>heat_space && heat_space>50)
	{
	target_shoot_rate_R=Shooter_R.cooling_rate/10;
	
	}
	else if(50>heat_space && heat_space>=5)
 {
	target_shoot_rate_R=Shooter_R.cooling_rate/10*0.8;
	
	}
 else
 {
 target_shoot_rate_R=0;
 }
	
motor_m2006[1].set_v=target_shoot_rate_R*K_shoot_rate_correct;//目标射频*射频修正系数
	
}

//==========================================拨盘堵转检测==========================================//
static void Bopan_judge()
{if(motor_m2006[0].tor_current>C_bopan_block_I)//修改堵转电流
	{bopan_reversal_flag_L=1;
	}
	else if(motor_m2006[0].tor_current<0 && motor_m2006[0].tor_current>-C_bopan_unblock_I)
	{bopan_reversal_flag_L=0;
	
	}		
	
	if(motor_m2006[1].tor_current>C_bopan_block_I)
	{bopan_reversal_flag_R=1;
	}
	else if(motor_m2006[1].tor_current<0 && motor_m2006[1].tor_current>-C_bopan_unblock_I)
	{bopan_reversal_flag_R=0;
	}		
	
}

