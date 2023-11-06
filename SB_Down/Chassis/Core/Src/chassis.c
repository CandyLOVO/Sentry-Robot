#include "freertos.h"
#include "can.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "pid.h"
#include "rc_potocal.h"
#include "handle_value.h"
#include "motion_overlay.h"

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;
extern int16_t motor_angle[4]; //6020角度
extern int16_t motor_speed[4]; //3508速度

//float* set;
//int16_t set[4];
int16_t theta = 60; //云台坐标系与底盘坐标系间夹角(此时为0~360度) 后期接收后需要对所得theta进行处理
int16_t get_6020[4];
uint16_t initial_angle[4];

void Chassis(void const * argument)
{
	pidTypeDef PID_angle[4];
	pidTypeDef PID_speed_3508[4];
	pidTypeDef PID_speed_6020[4];
	float PID_s[3] = {30,0.01,0};
	float PID_a[3] = {6,0,0.01};
	float PID[3] = {1,0,0};
	int16_t Max_out_a = 8192;
	int16_t Max_iout_a = 8192;
	int16_t Max_out_s = 30000; //电压控制转速，电流控制扭矩
	int16_t Max_iout_s = 30000;
	
	int16_t speed_6020[4];
	int16_t output_6020[4];
	int16_t output_3508[4];
	
	int m = 0;
	
	for(int i=0;i<4;i++){
		pid_init(&PID_speed_6020[i],PID_s[0],PID_s[1],PID_s[2]);
		pid_init(&PID_angle[i],PID_a[0],PID_a[1],PID_a[2]);
		pid_init(&PID_speed_3508[i],PID[0],PID[1],PID[2]);
	}
	
  for(;;)
  {
//**************************************读取6020初始角度**************************************//
//		if(m==0){
//			HAL_Delay(10);
//			for(int i=0;i<4;i++)
//			{
//				initial_angle[i] = motor[i].angle; //读取电机初始角度 0~8192
//			}
//			m++;
//		}
//********************************************************************************************//
		
		if(m==0){
			initial_angle[0] = 7819; //初始角度（底盘正前方各轮子角度）
			initial_angle[1] = 1858;
			initial_angle[2] = 3805;
			initial_angle[3] = 5735;
			m++;
		}
		
//		set = remote_value(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]); //遥控器当前角度——set 将遥控器希望转到的角度投影 0~180/0~-180
		
		compound_movement_6020(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]); //旋转加平移的6020角度设置 算得motor_angle[4]
		compound_movement_3508(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]); //旋转加平移的3508速度设置 算得motor_speed[4]
		HAL_Delay(10);
		for(int i=0;i<4;i++){
			
//			set[i] = compound_movement_6020(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1],i);
			
			get_6020[i] = - motor_value(initial_angle[i],motor[i].angle); //电机角度——get 将电机当前角度投影 0~180/0~-180  //逆时针旋转——加负号
			speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
			output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i].speed,speed_6020[i],Max_out_s,Max_iout_s);
		}
		
		translate_3508(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]);
		for(int i=0;i<4;i++){
			output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i+4].speed,motor_speed[i],Max_out_s,Max_iout_s); //3508速度控制
		}
		can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
		can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
    osDelay(10);
  }
}