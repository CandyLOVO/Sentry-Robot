#include "Yaw_task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "fdcan.h"
#include "main.h"
#include "pid_user.h"
#include "Exchange_task.h"
#include "imu_temp_ctrl.h"

//**************************************************变量定义**************************************************//
pidTypeDef pid_yaw_s_L;
pidTypeDef pid_yaw_a_L;
pidTypeDef pid_yaw_s_R;
pidTypeDef pid_yaw_a_R;
uint8_t can_send_data[8];
int32_t initial_angle_L;
int32_t initial_angle_R;
float yaw_angle_L;
float yaw_angle_R;
float target_yaw_a_L;
float target_yaw_s_L;
float target_yaw_a_R;
float target_yaw_s_R;
int16_t yaw_output_L;
int16_t yaw_output_R;
uint8_t warning_flag_L = 0; //判断当前角度是否处于危险角度的标志位
uint8_t warning_flag_R = 0;
int16_t up_limit;
int16_t low_limit;
uint8_t gimbal_control_6020[8];
uint8_t rotate_flag_L = 0; //判断是否需要反转
uint8_t rotate_flag_R = 0;

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;
extern receive_vision Rx_vision;
extern double yaw12;
extern int8_t flag;
//************************************************************************************************************//

//**************************************************任务实现**************************************************//

//黑头（左）ID为2，白头（右）ID为1
//两个头yaw用的FDCAN1,ID是1和2 【左右ID有待测试，需要更改motor[]中的序号，更改CAN发送电机控制数据的顺序】
void Yaw_Task(void * argument)
{
	Yaw_init();
//	osDelay(3000);
  for(;;)
  {
		//判断陀螺仪是否温补结束
		if(flag == 1)
		{
		//根据yaw的初始位置映射yaw的角度
		yaw_angle_L = -motor_value(initial_angle_L, motor[1].angle); //【要给负值！！】
		yaw_angle_R = -motor_value(initial_angle_R, motor[0].angle);
		
		//遥控器控小yaw模式，左->中，右->上
		if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==1)
		{
			target_yaw_a_L += rc_ctrl.rc.ch[2] * 0.2/660;
			target_yaw_a_R += rc_ctrl.rc.ch[0] * 0.2/660;
			
			yaw_control_L(-20, -160); //小yaw目标值软件限位
			yaw_control_R(160, 20);
		}
		
		//自瞄模式，左->下，右->下
		if(rc_ctrl.rc.s[1]==2 && rc_ctrl.rc.s[0]==2)
		{
			//四个摄像头都没有识别到
			if(Rx_vision.L_tracking==0 && Rx_vision.R_tracking==0 && Rx_vision.M_tracking==0)
			{
				//使小yaw开始正转
				if(rotate_flag_L == 0)
				{
					rotate_flag_L = 1;
				}
				if(rotate_flag_R == 0)
				{
					rotate_flag_R = 1;
				}
				
				//小yaw正反转巡航
				yaw_finding_L(-20, -160);
				yaw_finding_R(160, 20);
			}
			
			//左头识别到
			if(Rx_vision.L_tracking == 1)
			{
				rotate_flag_L = 0; //左头不转
				target_yaw_a_L = Rx_vision.L_yaw - yaw12; //左头目标值绝对坐标系转换
				yaw_control_L(-20, -160); //左头目标值软件限位
			}
			//右头识别到
			if(Rx_vision.R_tracking == 1)
			{
				rotate_flag_R = 0; //右头不转
				target_yaw_a_R = Rx_vision.R_yaw - yaw12; //右头目标值绝对坐标系转换
				yaw_control_R(160, 20); //右头目标值软件限位
			}
			//大yaw上的摄像头识别到
			if(Rx_vision.M_tracking == 1)
			{
				rotate_flag_L = 0;
				target_yaw_a_L = Rx_vision.L_yaw - yaw12;
				yaw_control_L(-20, -160);
				rotate_flag_R = 0;
				target_yaw_a_R = Rx_vision.R_yaw - yaw12;
				yaw_control_R(160, 20);
			}
		}
		
		//PID计算
		target_yaw_s_L = pid_cal_yaw_a(&pid_yaw_a_L, yaw_angle_L, target_yaw_a_L, warning_flag_L);
		yaw_output_L = pid_cal_s(&pid_yaw_s_L, motor[1].speed, target_yaw_s_L);
		target_yaw_s_R = pid_cal_yaw_a(&pid_yaw_a_R, yaw_angle_R, target_yaw_a_R, warning_flag_R);
		yaw_output_R = pid_cal_s(&pid_yaw_s_R, motor[0].speed, target_yaw_s_R);
		
		//小yaw6020电机报文发送 CAN1
//		gimbal_control_6020[0] = (yaw_output_L>>8)&0xff;
//		gimbal_control_6020[1] = yaw_output_L&0xff;
//		gimbal_control_6020[2] = (yaw_output_R>>8)&0xff;
//		gimbal_control_6020[3] = yaw_output_R&0xff;
		
		gimbal_control_6020[2] = (yaw_output_L>>8)&0xff;
		gimbal_control_6020[3] = yaw_output_L&0xff;
		gimbal_control_6020[0] = (yaw_output_R>>8)&0xff;
		gimbal_control_6020[1] = yaw_output_R&0xff;
		canx_send_data(&hfdcan1, 0x1FF, gimbal_control_6020, 8);
		osDelay(1);
		}
	}
}
//************************************************************************************************************//

//**************************************************函数实现**************************************************//
static void Yaw_init(void)
{
	//小yaw初始化
	initial_angle_L = 3646;
	initial_angle_R = 7830;
	target_yaw_a_L = 0;
	target_yaw_s_L = 0;
	target_yaw_a_R = 0;
	target_yaw_s_R = 0;
	
	//PID初始化
	pid_init(&pid_yaw_s_L,250,0.01,0,30000,30000); //PID初始化 PI
	pid_init(&pid_yaw_a_L,6,0,1,30000,30000); //PD
	
	pid_init(&pid_yaw_s_R,250,0.01,0,30000,30000); //PID初始化
	pid_init(&pid_yaw_a_R,6,0,1,30000,30000);
}

void yaw_control_L(int16_t max_angle, int16_t min_angle)
{
	//控制小yaw，max_angle和min_angle内的区域为禁区 （该软件限位针对于max_angle和min_angle都为负数的情况）
	//该函数需要配合pid_user.c的pid_cal_yaw_a函数使用
	
	//越界处理
	if(target_yaw_a_L>180)
	{
		target_yaw_a_L -= 360;
	} 
	if(target_yaw_a_L<-180)
	{
		target_yaw_a_L += 360;
	}
	
	//软件限位（目标值位于禁区内）
	if(target_yaw_a_L<0 && target_yaw_a_L>min_angle)
	{
		if(target_yaw_a_L <= max_angle)
		{
			target_yaw_a_L = max_angle;
		}
	}
	if((target_yaw_a_L>(-180)) && (target_yaw_a_L <max_angle))
	{
		if(target_yaw_a_L >= min_angle)
		{
			target_yaw_a_L = min_angle;
		}
	}
	
	//软件限位（当前值位于危险区内）
	if(((yaw_angle_L>0) && (yaw_angle_L<(min_angle+180))) || ((yaw_angle_L>max_angle) && (yaw_angle_L<0)))
	{
		up_limit = min_angle - max_angle;
		warning_flag_L = 1; //当前角度位于上方危险区域
	}
	else if(((yaw_angle_L>(max_angle+180)) && (yaw_angle_L<180)) || ((yaw_angle_L>-180) && (yaw_angle_L<min_angle)))
	{
		low_limit = max_angle - min_angle;
		warning_flag_L = 2; //当前角度位于下方危险区域
	}
	else
	{
		warning_flag_L = 0; //当前角度安全
	}
}

void yaw_control_R(int16_t max_angle, int16_t min_angle)
{
	//控制小yaw，max_angle和min_angle内的区域为禁区 （该软件限位针对于max_angle和min_angle都为正数的情况）
	//该函数需要配合pid_user.c的pid_cal_yaw_a函数使用
	
	//越界处理
	if(target_yaw_a_R>180)
	{
		target_yaw_a_R -= 360;
	}
	else if(target_yaw_a_R<-180)
	{
		target_yaw_a_R += 360;
	}
	
	//软件限位（目标值位于禁区内）
	if(target_yaw_a_R>0 && target_yaw_a_R<max_angle)
	{
		if(target_yaw_a_R >= min_angle)
		{
			target_yaw_a_R = min_angle;
		}
	}
	if(target_yaw_a_R<180 && target_yaw_a_R>min_angle)
	{
		if(target_yaw_a_R <= max_angle)
		{
			target_yaw_a_R = max_angle;
		}
	}
	
	//软件限位（当前值位于危险区内）
	if(((yaw_angle_R>0) && (yaw_angle_R<min_angle)) || ((yaw_angle_R>(max_angle-180)) && (yaw_angle_R<0)))
	{
		low_limit = max_angle - min_angle;
		warning_flag_R = 2; //当前角度位于上方危险区域
	}
	else if(((yaw_angle_R>max_angle) && (yaw_angle_R<180)) || ((yaw_angle_R>-180) && (yaw_angle_R<(min_angle-180))))
	{
		up_limit = min_angle - max_angle;
		warning_flag_R = 1; //当前角度位于下方危险区域
	}
	else
	{
		warning_flag_R = 0; //当前角度安全
	}
}

void yaw_finding_L(int16_t max_angle, int16_t min_angle)
{
	//左头巡航
	if(rotate_flag_L == 1)
	{
		target_yaw_a_L += 0.1; //以0.1/度的速度巡航
		if(target_yaw_a_L >= (min_angle+360))
		{
			rotate_flag_L = 2;
		}
	}
	else if(rotate_flag_L == 2) //左头反转
	{
		target_yaw_a_L -= 0.1;
		if(target_yaw_a_L <= max_angle)
		{
			rotate_flag_L = 1;
		}
	}
}

void yaw_finding_R(int16_t max_angle, int16_t min_angle)
{
	//右头巡航
	if(rotate_flag_R == 1)
	{
		target_yaw_a_R -= 0.1; //以0.1/度的速度巡航
		if(target_yaw_a_R <= (max_angle-360))
		{
			rotate_flag_R = 2;
		}
	}
	else if(rotate_flag_R == 2) //右头反转
	{
		target_yaw_a_R += 0.1;
		if(target_yaw_a_R >= min_angle)
		{
			rotate_flag_R = 1;
		}
	}
}

//void yaw_control_R(int16_t max_angle, int16_t min_angle)
//{
//	//控制小yaw，max_angle和min_angle内的区域为禁区 （该软件限位针对于max_angle和min_angle都为负数的情况）
//	//该函数需要配合pid_user.c的pid_cal_yaw_a函数使用
//	
//	//越界处理
//	if(target_yaw_a_R>180)
//	{
//		target_yaw_a_R -= 360;
//	}
//	else if(target_yaw_a_R<-180)
//	{
//		target_yaw_a_R += 360;
//	}
//	
//	//软件限位（目标值位于禁区内）
//	if((target_yaw_a_R>min_angle) && (target_yaw_a_R<max_angle))
//	{
//		target_yaw_a_R = 0;
//	}
//	
//	//软件限位（当前值位于危险区内）
//	if(((yaw_angle_R>0) && (yaw_angle_R<(min_angle+180))) || ((yaw_angle_R>max_angle) && (yaw_angle_R<0)))
//	{
//		up_limit = min_angle - max_angle;
//		warning_flag_R = 1; //当前角度位于上方危险区域
//	}
//	else if(((yaw_angle_R>(max_angle+180)) && (yaw_angle_R<180)) || ((yaw_angle_R>-180) && (yaw_angle_R<min_angle)))
//	{
//		low_limit = max_angle - min_angle;
//		warning_flag_R = 2; //当前角度位于下方危险区域
//	}
//	else
//	{
//		warning_flag_R = 0; //当前角度安全
//	}
//}

//void yaw_control_L(int16_t max_angle, int16_t min_angle)
//{
//	//控制小yaw，max_angle和min_angle内的区域为禁区 （该软件限位针对于max_angle和min_angle都为正数的情况）
//	//该函数需要配合pid_user.c的pid_cal_yaw_a函数使用
//	
//	//越界处理
//	if(target_yaw_a_L>180)
//	{
//		target_yaw_a_L -= 360;
//	}
//	else if(target_yaw_a_L<-180)
//	{
//		target_yaw_a_L += 360;
//	}
//	
//	//软件限位（目标值位于禁区内）
//	if((target_yaw_a_L>min_angle) && (target_yaw_a_L<max_angle))
//	{
//		target_yaw_a_L = 0;
//	}
//	
//	//软件限位（当前值位于危险区内）
//	if(((yaw_angle_L>0) && (yaw_angle_L<min_angle)) || ((yaw_angle_L>(max_angle-180)) && (yaw_angle_L<0)))
//	{
//		low_limit = max_angle - min_angle;
//		warning_flag_L = 2; //当前角度位于上方危险区域
//	}
//	else if(((yaw_angle_L>max_angle) && (yaw_angle_L<180)) || ((yaw_angle_L>-180) && (yaw_angle_L<(min_angle-180))))
//	{
//		up_limit = min_angle - max_angle;
//		warning_flag_L = 1; //当前角度位于下方危险区域
//	}
//	else
//	{
//		warning_flag_L = 0; //当前角度安全
//	}
//}

//void yaw_finding_R(int16_t max_angle, int16_t min_angle)
//{
//	//左头巡航
//	if(rotate_flag_R == 1)
//	{
//		target_yaw_a_R += 0.1; //以0.1/度的速度巡航
//		if(target_yaw_a_R >= (min_angle+360))
//		{
//			rotate_flag_R = 2;
//		}
//	}
//	else if(rotate_flag_R == 2) //左头反转
//	{
//		target_yaw_a_R -= 0.1;
//		if(target_yaw_a_R <= max_angle)
//		{
//			rotate_flag_R = 1;
//		}
//	}
//}

//void yaw_finding_L(int16_t max_angle, int16_t min_angle)
//{
//	//右头巡航
//	if(rotate_flag_L == 1)
//	{
//		target_yaw_a_L -= 0.1; //以0.1/度的速度巡航
//		if(target_yaw_a_L <= (max_angle-360))
//		{
//			rotate_flag_L = 2;
//		}
//	}
//	else if(rotate_flag_L == 2) //右头反转
//	{
//		target_yaw_a_L += 0.1;
//		if(target_yaw_a_L >= min_angle)
//		{
//			rotate_flag_L = 1;
//		}
//	}
//}

float motor_value(int16_t k, int16_t n)
{
	//将6020电机的角度以初始角度为0，映射到0~+-180度 (k:设定的初始角度“0” ； n:想要映射的角度)
	//该解算为倒装6020的映射 【正装6020需要将输出结果加“-”！！！！】
	float ans;
	if(k>=0 && k<4096){
		if(n>=0 && n<(k+4096)){
			n = k - n;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
		else if(n>=(k+4096) && n<8191){
			n = 8192 - n + k;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
	}
	
	else if(k>=4096 && k<8192){
		if(n>=0 && n<(k-4096)){
			n = -8192 + k - n;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
		else if(n>=(k-4096) && n<8191){
			n = k - n;
			ans = (float)n * 360.f / 8192.f;
			return ans;
		}
	}
}
//************************************************************************************************************//
