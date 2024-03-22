/*
*@author     CandyL
*@brief      底盘平移、旋转、平移+旋转控制代码 
*@date       2023.12.30
*@param      None
*@return     None
*@warning    计算3508与6020的PID
*/

#include "channel_changes.h"
#include "rc_potocal.h"
#include "chassis.h"
#include "motion_overlay.h"
#include "handle_value.h"
#include "user_can.h"
#include "user_pid.h"
#include "math.h"
#include "judge.h"

extern RC_ctrl_t rc_ctrl;
extern uint16_t initial_angle[4];
extern motor_info motor[8];
extern pidTypeDef PID_angle[4];
extern pidTypeDef PID_speed_3508[4];
extern pidTypeDef PID_speed_6020[4];
extern int16_t motor_angle[4]; //6020angle 设置为全局变量在"motion_overlay.c"中计算
extern int16_t motor_speed[4]; //3508speed
extern int16_t Max_out_a;
extern int16_t Max_iout_a;
extern int16_t Max_out_s;
extern int16_t Max_iout_s;
extern up_data Receive; //上C板数据

float Watch_Power_Max;
float Watch_Power;
float Watch_Buffer;
double Chassis_pidout;
double Chassis_pidout_target;
static double Scaling1=0,Scaling2=0,Scaling3=0,Scaling4=0;
float Klimit=1;
float Plimit=0;
float Chassis_pidout_max;
extern Sentry_t Sentry;

int16_t get_6020[4];
int16_t speed_6020[4];
int16_t output_6020[4];
int16_t output_3508[4];

//将3508和6020运动模式结合，形成底盘控制
 void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }

    }

}
static void Chassis_Power_Limit(double Chassis_pidout_target_limit)
{	
	//819.2/A，假设最大功率为120W，那么能够通过的最大电流为5A，取一个保守值：800.0 * 5 = 4000
	Watch_Power_Max=Klimit;	Watch_Power=Sentry.Myself_chassis_power;	Watch_Buffer=Sentry.Myself_chassis_power_buffer;//Hero_chassis_power_buffer;//限制值，功率值，缓冲能量值，初始值是1，0，0
	//get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//通过裁判系统和编码器值获取（限制值，实时功率，实时缓冲能量）

		Chassis_pidout_max=32768;//32768，40，960			15384 * 4，取了4个3508电机最大电流的一个保守值

		if(Watch_Power>450)	Motor_Speed_limiting(output_3508,4096);//限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变，不知道为啥一开始用的Power>960,可以观测下这个值，看看能不能压榨缓冲功率
	else{
		Chassis_pidout=(
						fabs(output_3508[0]-motor[0].speed)+
						fabs(output_3508[1]-motor[1].speed)+
						fabs(output_3508[2]-motor[2].speed)+
						fabs(output_3508[3]-motor[3].speed));//fabs是求绝对值，这里获取了4个轮子的差值求和
		
//	Chassis_pidout_target = fabs(motor_speed_target[0]) + fabs(motor_speed_target[1]) + fabs(motor_speed_target[2]) + fabs(motor_speed_target[3]);

		/*期望滞后占比环，增益个体加速度*/
		if(Chassis_pidout)
		{
		Scaling1=(output_3508[0]-motor[0].speed)/Chassis_pidout;	
		Scaling2=(output_3508[1]-motor[1].speed)/Chassis_pidout;
		Scaling3=(output_3508[2]-motor[2].speed)/Chassis_pidout;	
		Scaling4=(output_3508[3]-motor[3].speed)/Chassis_pidout;//求比例，4个scaling求和为1
		}
		else{Scaling1=0.25,Scaling2=0.25,Scaling3=0.25,Scaling4=0.25;}
		
		/*功率满输出占比环，车总增益加速度*/
//		if(Chassis_pidout_target) Klimit=Chassis_pidout/Chassis_pidout_target;	//375*4 = 1500
//		else{Klimit = 0;}
		Klimit = Chassis_pidout/Chassis_pidout_target_limit;
		
		if(Klimit > 1) Klimit = 1 ;
		else if(Klimit < -1) Klimit = -1;//限制绝对值不能超过1，也就是Chassis_pidout一定要小于某个速度值，不能超调

		/*缓冲能量占比环，总体约束*/
		if(Watch_Buffer<50&&Watch_Buffer>=40)	Plimit=0.8;		//近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
		else if(Watch_Buffer<40&&Watch_Buffer>=35)	Plimit=0.7;
		else if(Watch_Buffer<35&&Watch_Buffer>=30)	Plimit=0.5;
		else if(Watch_Buffer<30&&Watch_Buffer>=20)	Plimit=0.3;
		else if(Watch_Buffer<20&&Watch_Buffer>=10)	Plimit=0.25;
		else if(Watch_Buffer<10&&Watch_Buffer>=0)	Plimit=0.1;
		else {Plimit=1;}
//		float kpp = pid_cal_s(&Power,Hero_chassis_power,Hero_chassis_power_limit,Max_out_s,Max_iout_s);
//		Plimit = 0.5+kpp;
//		float pppp = (Hero_chassis_power_buffer-(Hero_chassis_power_limit-Hero_chassis_power)*0.01);
//		float ppout = pid_cal_s(&PID_speed_3508[0],pppp,20,Max_out_s,Max_iout_s);
//		Plimit += ppout;
//		if(Plimit<0)Plimit=0;
//				if(data[1]<24&&data[1]>23)	Plimit=0.9;		//近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
//		else if(data[1]<23&&data[1]>22)	Plimit=0.8;
//		else if(data[1]<22&&data[1]>20)	Plimit=0.7;
//		else if(data[1]<20&&data[1]>18)	Plimit=0.6;
//		else if(data[1]<18&&data[1]>16)	Plimit=0.5;
//		else if(data[1]<16)	Plimit=0.3;
//		else {Plimit=1;}
		
		output_3508[0] = Scaling1*(Chassis_pidout_max*Klimit)*Plimit;//输出值
		output_3508[1] = Scaling2*(Chassis_pidout_max*Klimit)*Plimit;
		output_3508[2] = Scaling3*(Chassis_pidout_max*Klimit)*Plimit;
		output_3508[3] = Scaling4*(Chassis_pidout_max*Klimit)*Plimit;/*同比缩放电流*/

	}

}
/********************************************************底盘平移控制********************************************************/
void translational_control()
{
	translate_6020(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3]); //还没有用导航数值
	translate_3508(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3]);
	
	for(int i=0;i<4;i++){
		//电机角度：get;将当前电机角度投影范围为 0 至 180/0 至 -180
		//加负号：使得电机解算方向与遥控器相符合（解算+-180时的小bug）
		get_6020[i] = -motor_value(initial_angle[i],motor[i+4].angle);
		
//		if((motor_angle[i]-get_6020[i])>90)
//		{
//			motor_angle[i] -= 180;
//			motor_speed[i] = -motor_speed[i];
//		}
//		else if((motor_angle[i]-get_6020[i])<-90)
//		{
//			motor_angle[i] += 180;
//			motor_speed[i] = -motor_speed[i];
//		}
		
		output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i].speed,motor_speed[i],Max_out_s,Max_iout_s);
		
		//PID计算(get,set)->(当前值，目标值)
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
	}
	
	for(int i=0;i<4;i++){
		
	}
	Chassis_Power_Limit(40000);
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
}
/****************************************************************************************************************************/


/***********************************************************底盘旋转控制*****************************************************/
void rotate_control()
{
	rotate_6020(); 
	for(int i=0;i<4;i++){
		get_6020[i] = -motor_value(initial_angle[i],motor[i+4].angle);
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
	}
	rotate_3508(rc_ctrl.rc.ch[4]); //滚轮控制旋转
	for(int i=0;i<4;i++){
		output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i].speed,motor_speed[i],Max_out_s,Max_iout_s);
	}
	Chassis_Power_Limit(40000);
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
}

void rotate_control_none()
{
	rotate_6020(); 
	for(int i=0;i<4;i++){
		get_6020[i] = -motor_value(initial_angle[i],motor[i+4].angle);
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
	}
	rotate_3508(500); //滚轮控制旋转
	for(int i=0;i<4;i++){
		output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i].speed,motor_speed[i],Max_out_s,Max_iout_s);
	}
	Chassis_Power_Limit(40000);
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
}
/****************************************************************************************************************************/


/******************************************************底盘旋转+平移控制*****************************************************/
void compound_control()
{
	//设置6020的旋转和平移角度，计算公式为motor_angle[4]
	compound_movement_6020(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3]); 
	//设置3508的旋转和平移速度，计算公式为motor_speed[4]
	compound_movement_3508(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3]);
	
	for(int i=0;i<4;i++){
		get_6020[i] = -motor_value(initial_angle[i],motor[i+4].angle);
		
//		if((motor_angle[i]-get_6020[i])>90)
//		{
//			motor_angle[i] -= 180;
//			motor_speed[i] = -motor_speed[i];
//		}
//		else if((motor_angle[i]-get_6020[i])<-90)
//		{
//			motor_angle[i] += 180;
//			motor_speed[i] = -motor_speed[i];
//		}
		
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
		
	}
	Chassis_Power_Limit(40000);
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
}
/****************************************************************************************************************************/

/*************************************************************导航控制*******************************************************/
void navigation_control()
{
	compound_movement_6020(Receive.nav_vx, Receive.nav_vy);
	compound_movement_3508(Receive.nav_vx, Receive.nav_vy);
	
	for(int i=0;i<4;i++){
		get_6020[i] = -motor_value(initial_angle[i],motor[i+4].angle);
//		if((motor_angle[i]-get_6020[i])>90)
//		{
//			motor_angle[i] -= 180;
//			motor_speed[i] = -motor_speed[i];
//		}
//		else if((motor_angle[i]-get_6020[i])<-90)
//		{
//			motor_angle[i] += 180;
//			motor_speed[i] = -motor_speed[i];
//		}
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
		output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i].speed,motor_speed[i],Max_out_s,Max_iout_s);
	}
	Chassis_Power_Limit(40000);
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
}
/****************************************************************************************************************************/
