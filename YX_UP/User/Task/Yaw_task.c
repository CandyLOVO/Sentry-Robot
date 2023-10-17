#include "Yaw_task.h"
#include "Exchange_task.h"
#include "main.h"

//================================================YAW轴电机控制任务================================================//

//	思路：利用速度环作基础PID反馈给云台，用云台陀螺仪的yaw值不变作角度环进行补偿
//  注意：云台往左转是负数，即顺时针编码器数值递增
//  这一版本是针对遥控器控制的版本
//  YAW轴是6020电机，采用上C板CAN_1，电机ID为6

//================================================全局变量================================================//
int yaw_fix_flag = 1;		//yaw锁定标识符，为1时不锁定，为0时锁定yaw
int8_t Update_yaw_flag;		//光电门更新yaw轴标志位(未使用)

fp32 ins_yaw;		//Imu实际角度值
fp32 target_yaw;	//目标角度值
fp32 err_yaw;		//角度差值
fp32 angle_weight = 1;	//角度环->速度环，映射的权重

//前馈控制变量
int16_t Rotate_w;
int16_t Rotate_W;

//================================================宏定义================================================//
#define Rotate_gain 1.23f		//角速度前馈控制映射权重
#define Chassis_R	30.0f		//车自转半径
#define Chassis_r 7.5f		//麦轮半径
#define valve 20		//遥控器阈值(死区)
#define base 1024		//遥控器的回中值
#define base_max 1684		//遥控器最大值
#define base_min 364		//遥控器最小值
#define angle_valve 1		//角度阈值，在这个范围内就不去抖动了(锁定yaw轴时使用)
#define mouse_x_valve 10		//鼠标控制阈值
#define mouse_x_weight 0.5f		//鼠标映射权重
#define Yaw_sita_weight 0.5f 		//遥控器控制位置环权重
#define Yaw_minipc_weight 1.75f		//视觉跟随映射权重(速度版)
#define Yaw_minipc_sita_weight 0.003f	//视觉跟随映射权重(位置版)

//================================================函数================================================//

//初始化PID参数
static void Yaw_init();	

//每次循环初始化
static void Yaw_loop_init();

//读取imu参数
static void Yaw_read_imu();

//陀螺仪锁云台(回中，速度环)
static void Yaw_fix();

//陀螺仪锁云台(回中，位置环)
static void Yaw_fix_sita();

//Mode_1下的控制算法,直接定速巡逻移动（速度环控制，编码器）
static void Yaw_mode_search();

//Mode_2下的控制算法，速度环控制（编码器）
static void Yaw_mode_remote_speed();

//Mode_3下的控制算法，位置环控制（陀螺仪）
static void Yaw_mode_remote_site();

//陀螺仪Yaw越界处理
static void detel_calc();

//角速度补偿前馈控制
static void Yaw_Rotate();

//鼠标控制叠加
static void Yaw_mouse();

//PID计算和发送
static void Yaw_can_send();

//叠加视觉自瞄(速度环)
static void Yaw_minipc_control();

//叠加视觉自瞄(位置环)
static void Yaw_minipc_control_sita();


//================================================YAW轴控制主函数================================================//
void Yaw_task(void const *pvParameters)
{
  //参数初始化设置
	Yaw_init();
	
	//循环任务运行
  for(;;)
  {
		Yaw_loop_init();//循环初始化
		Yaw_read_imu();//获取Imu角度

		//模式判断,左上角开关开到最下方
		if(rc_ctrl.rc.s[1] == 2 && ins_yaw)
		{
			if(foe_flag)	//如果视觉检测到目标
			{
					Yaw_Rotate();		//前馈控制补偿底盘带来的旋转角速度
					Yaw_fix_sita();		//角度控制
					Yaw_minipc_control_sita();	//视觉跟随
			}			
			else//没检测到开巡航模式
			{					
					Yaw_Rotate();			//前馈控制补偿底盘带来的旋转角速度	
					Yaw_mode_search();			//哨兵巡航模式
					yaw_fix_flag = 1;		//赋予不锁定的标志位
			}
		}
		else if(rc_ctrl.rc.s[1] == 1 || rc_ctrl.rc.s[1] == 3)
		{
			Yaw_Rotate();
			Yaw_mode_remote_site();		//遥控器控制模式(位置控制)
			Yaw_minipc_control_sita();	//视觉跟随
			yaw_fix_flag = 1;		//赋予不锁定的标志位
		}
		detel_calc();	//越界处理
		target_speed[6] +=  pid_calc_sita(&motor_pid_sita[6], target_yaw, ins_yaw);//角度->速度（内含越界处理）
		motor_info[6].set_voltage = pid_calc(&motor_pid[6], target_speed[6], 20 * ins_data.gyro[2]);//用陀螺仪的角速度（rad/s -> r/min），速度->电流
		Yaw_can_send();//电机电流数据发送
    osDelay(1);
  }

}


//初始化PID参数
static void Yaw_init()	
{
	//id为can1的5号
	pid_init(&motor_pid[6],350,0.01,0,30000,30000);
	pid_init(&motor_pid_sita[6],20,0,10,30000,30000);
	target_yaw = ins_yaw;
}


//================================================循环初始化================================================//
static void Yaw_loop_init()
{
	target_speed[6]=0;
}


//================================================获取imu角度================================================//
static void Yaw_read_imu()
{
		//三个角度值读取
		ins_yaw = ins_data.angle[0];
		//ins_pitch = ins_data.angle[1];
		//ins_row = ins_data.angle[2];
}

//================================================Yaw电机电流发送================================================//
static void Yaw_can_send()
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x2ff;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = 0x00;	//先发高八位		
  tx_data[1] = 0x00;
  tx_data[2] = (motor_info[6].set_voltage>>8)&0xff;	//先发高八位		
  tx_data[3] = (motor_info[6].set_voltage)&0xff;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}

//================================================YAW前馈补偿角速度================================================//
//这个算法重新写过一个麦轮解算后感觉可能会有点问题，但主要由反馈主导，前馈控制影响不大
//这里直接叠加到速度环了
static void Yaw_Rotate()
{
	Rotate_W = (Rotate_gain * Rotate_w * Chassis_r) / Chassis_R;
	target_speed[6] += Rotate_W;
}

//================================================陀螺仪锁yaw轴(未使用，供参考)================================================//
static void Yaw_fix()
{
	//摇杆回中锁云台(一般回中锁)
					if(yaw_fix_flag == 1)		//移动云台后重新记住云台的初始位置的值
					{
						target_yaw = ins_yaw;
						yaw_fix_flag = 0;
					}
						err_yaw = ins_yaw - target_yaw;		//用实时数据减初始数据
					
					//越界处理,保证转动方向不变
					if(err_yaw < -180)	//	越界时：180 -> -180
					{
						err_yaw += 360;
					}
					
					else if(err_yaw > 180)	//	越界时：-180 -> 180
					{
						err_yaw -= 360;
					}
				
					
					//阈值判断
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_speed[6] -= err_yaw * angle_weight;
					}
					
					else
					{
						target_speed[6] = 0;
					}
				
}

//================================================角度锁定================================================//
static void Yaw_fix_sita()
{
		if(yaw_fix_flag == 1)		//移动云台后重新记住云台的初始位置的值，循环执行这个模式的时候只会执行一次
		{
			target_yaw = ins_yaw;
			yaw_fix_flag = 0;
		}
}

//================================================鼠标控制================================================//
static void Yaw_mouse()
{
	if(mouse_x > mouse_x_valve || mouse_x < -mouse_x_valve)
	{
		yaw_fix_flag = 1;
		target_speed[6] -= (fp32)mouse_x * mouse_x_weight;
	}
}

//================================================定速巡航控制模式================================================//
static void Yaw_mode_search()
{
	target_yaw -= 0.03f;
}

//================================================速度控制模式================================================//
static void Yaw_mode_remote_speed()
{
	
		if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
	{
		Yaw_fix();
	}
	else if( (rc_ctrl.rc.ch[0] >= base+valve && rc_ctrl.rc.ch[0] <= base_max) || (e_flag) )
	{
		target_speed[6] -= 60;
		yaw_fix_flag = 1;
	}
	
	else if( (rc_ctrl.rc.ch[0] >= base_min && rc_ctrl.rc.ch[0]<base - valve ) || (q_flag) )
	{
		target_speed[6] += 60;
		yaw_fix_flag = 1;
	}
}

//================================================速度视觉跟随================================================//
static void Yaw_minipc_control()
{
		target_speed[6] -= ((fp32)Yaw_minipc_fp) * Yaw_minipc_weight;
}

//================================================位置控制模式================================================//
static void Yaw_mode_remote_site()
{
		if(rc_ctrl.rc.ch[0] >= 324 && rc_ctrl.rc.ch[0]<= 1684)
		{			
			target_yaw -= (rc_ctrl.rc.ch[0]-base)/660.0 * Yaw_sita_weight; 			
		}
}

//================================================越界处理================================================//
static void detel_calc()
{
	if(target_yaw >360)
	{
		target_yaw -=360;
	}
	
	else if(target_yaw<0)
	{
		target_yaw += 360;
	}
}

//================================================位置视觉跟随================================================//
//注意使用浮点数才能保留精度，解码和视觉对接好
//直接叠加角度环控制
static void Yaw_minipc_control_sita()
{
		target_yaw -= ((fp32)Yaw_minipc_fp) * Yaw_minipc_sita_weight;
}