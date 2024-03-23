#include "motion_overlay.h"
#include "handle_value.h"
#include "math.h"
#include "chassis.h"
#include "user_can.h"

#define cosin 0.707106781187 //二分之根号二
#define omega 25 //旋转叠加计算中的角速度 【严禁给15以上!!! yaw的6020给转烂了！！！！！！！！！】
#define radius 313.487 //舵轮距离车体中心的距离 杰舵248.248mm 哨兵313.487mm
extern fp32 error_theta;
extern motor_info motor[8];

int16_t motor_angle[4]; //需要6020转到的角度
int16_t motor_speed[4];

float rcLfFiter(float pre, float val,float value)
{
	val=((float)pre*value+val*(1-value));
  return val;
}

//3508和6020各运动方式的分别控制

/**********************************************************仅旋转****************************************************************/
//仅旋转的3508速度
void rotate_3508(int16_t vw)
{
	for(int i=0;i<4;i++)
	{
		if(motor_speed[i]<(vw*(9000/660)))
			motor_speed[i] += 80;
		else
			motor_speed[i] = vw*(9000/660);
	}
}	

//仅旋转的6020角度
void rotate_6020()
{
	motor_angle[0] = remote_value((+ omega*radius*cosin), ( + omega*radius*cosin));
	motor_angle[1] = remote_value((+ omega*radius*cosin), ( - omega*radius*cosin));
	motor_angle[2] = remote_value((- omega*radius*cosin), ( - omega*radius*cosin));
	motor_angle[3] = remote_value((- omega*radius*cosin), ( + omega*radius*cosin));
}
/********************************************************************************************************************************/


/**********************************************************仅平移****************************************************************/
//仅平移的3508速度
void translate_3508(int16_t x,int16_t y)
{
	motor_speed[0] =  sqrt(pow((float)x,2) + pow((float)y,2))*(3000/660);
	motor_speed[1] =  sqrt(pow((float)x,2) + pow((float)y,2))*(3000/660);
	motor_speed[2] =  sqrt(pow((float)x,2) + pow((float)y,2))*(3000/660);
	motor_speed[3] =  sqrt(pow((float)x,2) + pow((float)y,2))*(3000/660);
}

//仅平移的6020角度
void translate_6020(int16_t x,int16_t y)
{ 
	int16_t vx = x*cos(error_theta) - y*sin(error_theta);
	int16_t vy = x*sin(error_theta) + y*cos(error_theta);
//	for(int i=0;i<4;i++){
//		//将遥控器希望转到的角度投影 0~180/0~-180
//		motor_angle[i] = remote_value((float)vx , (float)vy);
//	}
	motor_angle[0] = remote_value((float)vx , (float)vy);
	motor_angle[1] = remote_value((float)vx , (float)vy);
	motor_angle[2] = remote_value((float)vx , (float)vy);
	motor_angle[3] = remote_value((float)vx , (float)vy);
}
/********************************************************************************************************************************/


/***********************************************************旋转+平移************************************************************/
//旋转+平移的3508速度
void compound_movement_3508(int16_t x,int16_t y)
{
	int16_t vx = (x*cos(error_theta) - y*sin(error_theta))*(3000/660); //底盘跟随云台处理后的x、y方向的速度
	int16_t vy = (x*sin(error_theta) + y*cos(error_theta))*(3000/660);
	
	//两个3508朝内 两个3508朝外
	if(motor_speed[0]<sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2)))
		motor_speed[0] += 80;
	else
		motor_speed[0] =  sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2)); //平移与旋转叠加后的向量长度
	
	if(motor_speed[1]<sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2)))
		motor_speed[1] += 80;
	else
		motor_speed[1] =  sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
	
	if(motor_speed[2]<sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2)))
		motor_speed[2] += 80;
	else
		motor_speed[2] =  sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
		
	if(motor_speed[3]<sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2)))
		motor_speed[3] += 80;
	else
		motor_speed[3] =  sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2));
}

//旋转+平移的6020角度
void compound_movement_6020(int16_t x,int16_t y)
{
	int16_t vx = (x*cos(error_theta) - y*sin(error_theta))*(3000/660);
	int16_t vy = (x*sin(error_theta) + y*cos(error_theta))*(3000/660);
	
	//两个3508朝内 两个3508朝外
	motor_angle[0] = remote_value(((float)vx + omega*radius*cosin), ((float)vy + omega*radius*cosin));
	motor_angle[1] = remote_value(((float)vx + omega*radius*cosin), ((float)vy - omega*radius*cosin));
	motor_angle[2] = remote_value(((float)vx - omega*radius*cosin), ((float)vy - omega*radius*cosin));
	motor_angle[3] = remote_value(((float)vx - omega*radius*cosin), ((float)vy + omega*radius*cosin));
}
/********************************************************************************************************************************/
