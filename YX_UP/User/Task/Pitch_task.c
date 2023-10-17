#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "INS_task.h"
#include "Pitch_task.h"
#include "Exchange_task.h"

//第一版：
//此任务用来对云台进行模式选择，控制，校准等
//封装一些函数用来进行控制调用
//Pitch采用上C板CAN_2，电机ID为5
//存在问题：需要将云台Pitch锁在最下方再启动，Pitch受高频噪点影响，收敛速度太慢

//第二版：
//采用双6020结构，新加的Pitch走上C板CAN_2，电机ID为6

//定义一些变量
//限位参数（机械测量）
#define Up_inf 35
#define Down_inf 0
#define mouse_y_valve 10
#define mouse_y_weight 12.0f
#define Pitch_minipc_valve 0.5f
#define Pitch_minipc_weight	0.5f
#define Pitch_sita_weight 0.2f
#define Pitch_sita_minipc_weight 0.002f


#define Pitch_up 4500
#define Pitch_down 4025

//imu数据
fp32 Err_pitch;
int16_t Up_pitch;
int16_t Down_pitch;
uint16_t Remember_pitch = 0;
uint8_t Remember_pitch_flag = 1;
extern ins_data_t ins_data;
extern int16_t mouse_y;

uint16_t gimbal_rotor_angle;
uint16_t gimbal_rotor_angle_2;
float Pitch_imu;
float Pitch_imu_speed;
float init_pitch;
float ins_pitch_speed;

//定时器计数器
uint16_t TIM1_Count = 0;
uint8_t TIM1_Mode = 1;

//初始化PID参数
static void gimbal_init();	

//初始清零
static void gimbal_zero();

//校验连接成功
static bool gimbal_judge();	

//读取imu参数
static void gimbal_read_imu();

//读取编码器的值
static void gimbal_read();

//模式选择
static void gimbal_choice();

//Mode_1下的控制算法(速度环)
static void gimbal_mode_1();

//Mode_2下的巡逻控制算法(速度环)
static void gimbal_mode_2();

//Mode_3下的控制算法(位置环--->imu)
static void gimbal_mode_3();

//Mode_4下的巡逻控制算法(位置环)
static void gimbal_mode_4();

//Mode_4下的控制算法(位置环--->编码器)
static void gimbal_mode_5();

//PID计算和发送
static void gimbal_can_send();

//限位（编码器）
static void gimbal_limit();

//限位（陀螺仪）
static void gimbal_imu_limit();

//鼠标控制Pitch(叠加)
static void gimbal_mouse();

//叠加自瞄
static void gimbal_minipc_control();

//叠加自瞄(位置环)
static void gimbal_minipc_sita_control();

// pitch

void Pitch_task(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */

	gimbal_init();	//PID参数调整
	
  for(;;)
  {
		gimbal_zero();
		gimbal_read();
		gimbal_read_imu();
		if(rc_ctrl.rc.s[1]==3 || rc_ctrl.rc.s[1]==1	)
		{
			gimbal_mode_3();	
		  gimbal_minipc_sita_control();
		 }
		  
		else if(rc_ctrl.rc.s[1]==2)		//哨兵测试，左上角拨到最下端启动
		{
			if(foe_flag)	//如果检测到目标
			{
				gimbal_minipc_sita_control(); 
			}
			
			else
			{
				gimbal_mode_4();		
			}
		}
		//gimbal_imu_limit();
//		target_speed_can_2[4] -= pid_calc_sita(&motor_pid_sita_can_2[4], init_pitch, ins_data.angle[1]);
		target_speed_can_2[5] += pid_calc_sita(&motor_pid_sita_can_2[5], init_pitch, ins_data.angle[1]);
//		target_speed_can_2[4] -= pid_calc_sita(&motor_pid_sita_can_2[4], init_pitch+3632, motor_info_can_2[4].rotor_angle);
		//target_speed_can_2[5] += pid_calc_sita(&motor_pid_sita_can_2[5], init_pitch, motor_info_can_2[5].rotor_angle);
		//motor_info_can_2[4].set_voltage = pid_calc(&motor_pid_can_2[4], target_speed_can_2[4], Pitch_imu_speed * -20);
//		if(target_speed_can_2[5] > 100)
//		{
//			target_speed_can_2[5] = 100;
//		}
//		else if(target_speed_can_2[5] < -100)
//		{
//			target_speed_can_2[5] = -100;
//		}
		ins_pitch_speed = ins_data.gyro[1] * 20;
		motor_info_can_2[5].set_voltage = pid_calc(&motor_pid_can_2[5], target_speed_can_2[5], ins_pitch_speed );
//		motor_info_can_2[4].set_voltage = -motor_info_can_2[5].set_voltage;
//		motor_info_can_2[4].set_voltage = pid_calc(&motor_pid_can_2[4], target_speed_can_2[4], motor_info_can_2[4].rotor_speed);
//		motor_info_can_2[5].set_voltage = pid_calc(&motor_pid_can_2[5], target_speed_can_2[5], motor_info_can_2[5].rotor_speed);
		gimbal_can_send();		
		
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

//初始化PID参数
static void gimbal_init()	
{
	//pid_init(&motor_pid_can_2[4],450,0.01,0,30000,30000);// 120 0.01 0
	pid_init(&motor_pid_can_2[5],250,0.01,0,30000,30000);// 120 0.01 0
	//pid_init(&motor_pid_sita_can_2[4],10,0,200,30000,30000);// 10 0 1300
	pid_init(&motor_pid_sita_can_2[5],5,0,100,30000,30000);// 10 0 1300
//	pid_init(&motor_pid_can_2[4],20,0,0,30000,30000);
//	pid_init(&motor_pid_can_2[5],20,0,0,30000,30000);
//	pid_init(&motor_pid_sita_can_2[4],10,0,1,30000,30000);
//	pid_init(&motor_pid_sita_can_2[5],10,0,1,30000,30000);
	init_pitch = Pitch_imu;
}


//校验连接成功
static bool gimbal_judge()
{

}


//编码器的值
static void gimbal_read()
{
	gimbal_rotor_angle = motor_info_can_2[4].rotor_angle;	//最下方:0E30   最上方:10B0  换算:3632->4272  ,步长：640
	gimbal_rotor_angle_2 = motor_info_can_2[5].rotor_angle; //最下方:09AA   最上方:0730   换算:2474->1840  ,步长：634
}

//陀螺仪的值
static void gimbal_read_imu()
{
	Pitch_imu = ins_data.angle[1];   //陀螺仪Pitch值
	Pitch_imu_speed = ins_data.gyro[1];   //陀螺仪角速度值
}


//模式选择
static void gimbal_choice()
{

}


//Mode_1算法，最简单的云台控制（速度环）
static void gimbal_mode_1()
{
		if( (rc_ctrl.rc.ch[1]>1074&&rc_ctrl.rc.ch[1]<=1684 ) || (ctrl_flag))
		{
			target_speed_can_2[4]=15;
			target_speed_can_2[5]=-15;
		}
		else if( (rc_ctrl.rc.ch[1]>=324&&rc_ctrl.rc.ch[1]<974) || (shift_flag))
		{
			target_speed_can_2[4]=-15;
			target_speed_can_2[5]=15;
		}
		else
		{
			target_speed_can_2[4]=0;
			target_speed_can_2[5]=0;
		}
}	

//PID计算和发送
static void gimbal_can_send()
{
		
  
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x2ff;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = (motor_info_can_2[4].set_voltage>>8)&0xff;	//先发高八位		
  tx_data[1] = (motor_info_can_2[4].set_voltage)&0xff;
  tx_data[2] = (motor_info_can_2[5].set_voltage>>8)&0xff;
  tx_data[3] = (motor_info_can_2[5].set_voltage)&0xff;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}


//陀螺仪限位（相对）,停止限位
static void gimbal_limit()
{
	gimbal_read_imu();
	if( gimbal_rotor_angle >= Pitch_up)
	{
		
		target_speed_can_2[4]=-20;
		target_speed_can_2[5]=20;
	}
	
	else if( gimbal_rotor_angle <= Pitch_down)
	{
		target_speed_can_2[4]=20;
		target_speed_can_2[5]=-20;
	}
}

static void gimbal_imu_limit()
{
	if(init_pitch < -25)
	{
		init_pitch = -25;
	}
	else if(init_pitch >= -5 && Pitch_imu)
	{
		init_pitch = -5;
	}
}



//鼠标控制
static void gimbal_mouse()
{
	if(mouse_y > mouse_y_valve || mouse_y < -mouse_y_valve)
	{
		target_speed_can_2[4] += (fp32)mouse_y * mouse_y_weight;
		target_speed_can_2[5] -= (fp32)mouse_y * mouse_y_weight;
	}
}

//自瞄
static void gimbal_minipc_control()
{
	if((fp32)(Pitch_minipc) > Pitch_minipc_valve || (fp32)(Pitch_minipc) < -Pitch_minipc_valve)
	{
		target_speed_can_2[4] -= ((fp32)Pitch_minipc_fp) * Pitch_minipc_weight;
		target_speed_can_2[5] += ((fp32)Pitch_minipc_fp) * Pitch_minipc_weight;
	}
}

static void gimbal_mode_2()
{
	switch(TIM1_Mode)
	{
		case 1: target_speed_can_2[4]=-4 , target_speed_can_2[5]=4;break;
		case 2: target_speed_can_2[4]=4 , target_speed_can_2[5]=-4;break;
	}
}

static void gimbal_zero()
{
	target_speed_can_2[4] = 0;
	target_speed_can_2[5] = 0;
}

static void gimbal_mode_3()
{
		if( (rc_ctrl.rc.ch[1]>=324 && rc_ctrl.rc.ch[1] <=1684 ) )
		{
			init_pitch -= (rc_ctrl.rc.ch[1]- 1024)/660.0 * Pitch_sita_weight; 			
		}
}

static void gimbal_minipc_sita_control()
{
		init_pitch -= ((fp32)Pitch_minipc_fp) * Pitch_sita_minipc_weight;
}

static void gimbal_mode_4()
{
	switch(TIM1_Mode)
	{
		case 1: init_pitch -= 0.03f;break;
		case 2: init_pitch += 0.03f;break;
	}
}

static void gimbal_mode_5()
{
		if( (rc_ctrl.rc.ch[1]>=324 && rc_ctrl.rc.ch[1] <=1684 ) )
		{
			init_pitch -= (rc_ctrl.rc.ch[1]- 1024)/660.0 * Pitch_sita_weight * 24.0f; 			
		}
}