#include "Friction_task.h"

//================================================发射机构控制任务================================================//
//第一版：
//这个用来写摩擦轮和拨盘
//摩擦轮的ID分别是1和2   ---   上C板CAN_2
//拨盘的ID是5   ---   CAN_1（上下联通）

//第二版：
//这个新增两个摩擦轮和拨盘
//新摩擦轮ID分别是3和4   ---   上C板CAN_2
//新增拨盘，拨盘ID是6   ---   CAN_1（上下联通）

//===============================================全局变量================================================//
int16_t bopan_shoot_speed = 90*36;	//拨盘发射弹丸转速
int16_t bopan_reversal_speed = -35*36;	//拨盘反转转速
uint8_t bopan_reversal_flag = 0;	//拨盘反转标志位，0为需要不反转，1为需要反转

//PID初始化
static void Friction_init();

//摩擦轮加速赋值
static void Friction_calc();

//摩擦轮减速赋值
static void Friction_down();

//摩擦轮充能判断
static bool Friction_judge();

//摩擦轮Pid输出值发送
static void Friction_send();

//拨盘Pid输出值计算和发送
static void Bopan_send(int16_t speed);

void Friction_task(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	
	Friction_init();	//PID初始化
	
  for(;;)
  {
		//===============================================摩擦轮================================================//
		if(rc_ctrl.rc.s[0] == 2)	//开启摩擦轮
		{
			Friction_calc();	//转速->电流
		}
		else
		{
			Friction_down();	//摩擦轮减速到零
		}
		Friction_send();	//摩擦轮电流发送
		
		//===============================================拨盘================================================//
		if(rc_ctrl.rc.s[1] == 1)	//开启拨盘(测试模式)
		{	
			if(!bopan_reversal_flag)	//拨盘正转
			{
				Bopan_send(bopan_shoot_speed);
			}
			else if(bopan_reversal_flag)	//拨盘反转
			{
				Bopan_send(bopan_reversal_speed);
			}
			
		}
		else if(rc_ctrl.rc.s[1] == 2  && foe_flag && (Yaw_minipc_fp<5.0f && Yaw_minipc_fp> -5.0f) && (Pitch_minipc_fp<5.0f && Pitch_minipc_fp>-5.0f) && Pitch_minipc_fp!=0 && Yaw_minipc_fp!=0)//检测到目标
		{
			if(!bopan_reversal_flag)	//拨盘正转
			{
				Bopan_send(bopan_shoot_speed);
			}
			else if(bopan_reversal_flag)	//拨盘反转
			{
				Bopan_send(bopan_reversal_speed);
			}
		}
		else
		{			
			Bopan_send(0);
		}
		
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

//===============================================PID初始化================================================//
static void Friction_init()
{
	pid_init(&motor_pid_can_2[0],40,0.8,1,16384,16384);//摩擦轮
	pid_init(&motor_pid_can_2[1],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[2],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[3],40,0.8,1,16384,16384);
	
	pid_init(&motor_pid[4],20,0.03,0.5,16384,16384);//拨盘(拨爪)
	pid_init(&motor_pid[5],20,0.03,0.5,16384,16384);
}

//===============================================摩擦轮转速->电流================================================//
static void Friction_calc()
{
	target_speed_can_2[0]=-19*350;
	target_speed_can_2[1]=19*350;
	target_speed_can_2[2]=-19*350;
	target_speed_can_2[3]=19*350;
	
	motor_info_can_2[0].set_voltage = pid_calc(&motor_pid_can_2[0], target_speed_can_2[0], motor_info_can_2[0].rotor_speed);
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[1].rotor_speed);
	motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed);
	motor_info_can_2[3].set_voltage = pid_calc(&motor_pid_can_2[3], target_speed_can_2[3], motor_info_can_2[3].rotor_speed);
}

//===============================================摩擦轮减速到零================================================//
static void Friction_down()
{
	target_speed_can_2[0]=0;
	target_speed_can_2[1]=0;
	target_speed_can_2[2]=0;
	target_speed_can_2[3]=0;
	motor_info_can_2[0].set_voltage = pid_calc(&motor_pid_can_2[0], target_speed_can_2[0], motor_info_can_2[0].rotor_speed);
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[1].rotor_speed);
	motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed);
	motor_info_can_2[3].set_voltage = pid_calc(&motor_pid_can_2[3], target_speed_can_2[3], motor_info_can_2[3].rotor_speed);
}

//===============================================摩擦轮转速判断(未使用)================================================//
static bool Friction_judge()
{
	if(	(motor_info_can_2[1].rotor_speed>=8500) && (motor_info_can_2[2].rotor_speed<=-8500) )
	{
		return true;
	}
	return false;
}

//===============================================摩擦轮电流发送函数================================================//
static void Friction_send()
{
		set_motor_voltage_can_2(0, 
                      motor_info_can_2[0].set_voltage, 
                      motor_info_can_2[1].set_voltage, 
                      motor_info_can_2[2].set_voltage, 
                      motor_info_can_2[3].set_voltage);
}

//===============================================拨盘电流发送函数================================================//
static void Bopan_send(int16_t speed)
{
		motor_info[4].set_voltage=pid_calc(&motor_pid[4],speed,motor_info[4].rotor_speed);
		motor_info[5].set_voltage=pid_calc(&motor_pid[5],-speed,motor_info[5].rotor_speed);
		set_motor_voltage(1, 
                      motor_info[4].set_voltage, 
                      motor_info[5].set_voltage, 
                      motor_info[6].set_voltage, 
                      0);
}
