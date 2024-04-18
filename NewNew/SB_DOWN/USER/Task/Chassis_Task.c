#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "pid_user.h"
#include "can_user.h"
#include "rc_potocal.h"
#include "struct_typedef.h"
#include "math.h"

/*********************************************************变量定义*********************************************************/
#define radius 3.075 // 615mm/2 m
#define cosin 0.707106781187 //二分之根号二

pidTypeDef pid_3508;
fp32 error_theta; //云台坐标系与底盘坐标系间夹角(此时为0~360度) 后期接收后需要对所得theta进行处理
int8_t omega = 0; //旋转叠加计算中的角速度 rad/min
int16_t target_speed[4]; //3508目标速度
int16_t out_speed[4]; //控制电流值

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;
extern up_data Receive;
/**************************************************************************************************************************/

/*********************************************************函数定义*********************************************************/
void task_init(void);
void Yaw_Diff(void);
void chassis_calculate(int16_t x, int16_t y);
/**************************************************************************************************************************/

/*******************************************************底盘控制任务*******************************************************/
void Chassis_Task(void const * argument)
{
	task_init();
	
  for(;;)
  {
		//计算底盘与云台差角
		Yaw_Diff();
		
		//遥控器控制模式，左->中间，右->中间
		if(rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==3)
		{
			omega = rc_ctrl.rc.ch[4]*0.05; //拨轮控制小陀螺
//			chassis_calculate(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]); //右拨杆控制底盘 遥控器右拨杆有问题
			chassis_calculate(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3]); //左拨杆控制底盘
			for(int i=0;i<4;i++)
			{
				out_speed[i] = pid_cal_s(&pid_3508, motor[i].speed, target_speed[i]);
			}
			can_cmd_send_3508(out_speed[0], out_speed[1], out_speed[2], out_speed[3]);
		}
		
		//导航上场模式，左->最下，右->最下
		else if(rc_ctrl.rc.s[0]==2 && rc_ctrl.rc.s[1]==2)
		{
			omega = 25; //给定小陀螺转速
			chassis_calculate(Receive.nav_vx, Receive.nav_vy); //输入导航x、y值，CAN1传来
			for(int i=0;i<4;i++)
			{
				out_speed[i] = pid_cal_s(&pid_3508, motor[i].speed, target_speed[i]);
			}
			can_cmd_send_3508(out_speed[0], out_speed[1], out_speed[2], out_speed[3]);
		}
		
    osDelay(1);
  }
}
/**************************************************************************************************************************/

/*********************************************************函数实现*********************************************************/
void task_init()
{
	//PID参数初始化
	pid_init(&pid_3508,10,0,0,30000,30000);
}

void Yaw_Diff()
{
	//计算底盘与云台间的相差角度
	error_theta = -Receive.yaw_value+180; //计算云台与底盘的夹角，使用9025编码值【底盘传来0~180、0~-180】
	error_theta = error_theta*3.1415926/180; //转化为弧度制
}

void chassis_calculate(int16_t x, int16_t y)
{
	//底盘跟随云台，乘旋转矩阵
	int16_t vx = x*cos(error_theta) - y*sin(error_theta); //RPM
	int16_t vy = x*sin(error_theta) + y*cos(error_theta);
	//全向轮运动解算
	target_speed[0] = omega*radius + vx*cosin - vy*cosin;
	target_speed[1] = omega*radius - vx*cosin - vy*cosin;
	target_speed[2] = omega*radius - vx*cosin + vy*cosin;
	target_speed[3] = omega*radius + vx*cosin + vy*cosin;
}
/*************************************************************************************************************************/
