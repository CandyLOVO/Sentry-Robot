/*
*@author     CandyL
*@brief      舵轮底盘控制代码  
*@date       2023.12.30
*@param      None
*@return     None
*@warning    该文件为主函数，功能可根据需求修改，具体功能函数在"motion_overlay.c"中。
*/

#include "freertos.h"
#include "can.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "user_pid.h"
#include "rc_potocal.h"
#include "handle_value.h"
#include "motion_overlay.h"
#include "channel_changes.h"
#include "INS_task.h"

extern motor_info motor[8]; //底盘电机数据
extern RC_ctrl_t rc_ctrl; //遥控器数据
extern fp32 INS_angle[3]; //下C板陀螺仪数据
extern up_data Receive; //上C板数据
extern int16_t motor_angle[4]; //6020角度 在motion_overlay.c中计算 作为全局变量
extern int16_t motor_speed[4]; //3508速度

uint16_t initial_angle[4];
int16_t Max_out_a = 20000;
int16_t Max_iout_a = 20000;
int16_t Max_out_s = 16384; //电压控制转速，电流控制扭矩
int16_t Max_iout_s = 2000;
pidTypeDef PID_angle[4];
pidTypeDef PID_speed_3508[4];
pidTypeDef PID_speed_6020[4];
fp32 error_theta; //云台坐标系与底盘坐标系间夹角(此时为0~360度) 后期接收后需要对所得theta进行处理

//PID初始化
void Chassis_init(void);

//得到上C板与下C板间yaw的差值
void Yaw_Diff(void);

void Chassis(void const * argument)
{
	Chassis_init();
	
	//设置初始角度		
	initial_angle[0] = 1681; //初始角度（底盘正前方各轮子角度）
	initial_angle[1] = 3790;
	initial_angle[2] = 1697;
	initial_angle[3] = 3806;

  for(;;)
  {
		Yaw_Diff(); //得到上C板与下C板间yaw的差值

		//遥控器控制底盘不同运动
		//具体实现方式在"motion_overlay.c"
		
		if(rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==1)
		{
			if(rc_ctrl.rc.ch[4]==0)
			{
				translational_control(); //平移运动
			}
			else
			{
				rotate_control(); //旋转运动
			}
		}
		
		else if(rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==3)
		{
			compound_control(); //旋转加平移运动
		}
    osDelay(2);
  }
}

void Chassis_init()
{
	float PID_s[3] = {10,0.05,0};
	float PID_a[3] = {35,0,3};
	float PID[3] = {10,0.2,0};
	
	for(int i=0;i<4;i++){
		pid_init(&PID_speed_6020[i],PID_s[0],PID_s[1],PID_s[2]);
		pid_init(&PID_angle[i],PID_a[0],PID_a[1],PID_a[2]);
		pid_init(&PID_speed_3508[i],PID[0],PID[1],PID[2]);
	}
}

void Yaw_Diff()
{
	error_theta = Receive.yaw_value; //计算云台与底盘的夹角，后使用9025编码值【底盘传来0~180、0~-180】
	error_theta = error_theta*3.1415926/180; //转化为弧度制
}