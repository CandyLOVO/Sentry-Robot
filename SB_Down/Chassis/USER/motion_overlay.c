#include "motion_overlay.h"
#include "handle_value.h"
#include "math.h"
#include "chassis.h"

#define cosin 0.707106781187 //二分之根号二
#define omega 13 //旋转叠加计算中的角速度
#define radius 50 //舵轮距离车体中心的距离

extern int16_t theta;

int16_t motor_angle[4];
int16_t motor_speed[4];

void translate_3508(int16_t x,int16_t y) //仅平移的3508速度、仅旋转的3508速度
{
	for(int i=0;i<4;i++){
		motor_speed[i] = sqrt(pow((float)x,2) + pow((float)y,2)); //将杆量反映到3508的速度
	}
}	

void translate_6020(int16_t x,int16_t y) //仅平移的6020角度
{
	int16_t vx = x*cos(theta) - y*sin(theta);
	int16_t vy = x*sin(theta) + y*cos(theta);
	for(int i=0;i<4;i++){
		motor_angle[i] = remote_value((float)vx , (float)vy); //将遥控器希望转到的角度投影 0~180/0~-180
	}
}

void compound_movement_3508(int16_t x,int16_t y) //旋转+平移的3508速度
{
	int16_t vx = x*cos(theta) - y*sin(theta);
	int16_t vy = x*sin(theta) + y*cos(theta);
	motor_speed[0] = sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2));
	motor_speed[1] = sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2));
	motor_speed[2] = sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
	motor_speed[3] = sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
}

void compound_movement_6020(int16_t x,int16_t y) //旋转+平移的6020角度
{
	int16_t vx = x*cos(theta) - y*sin(theta);
	int16_t vy = x*sin(theta) + y*cos(theta);
	motor_angle[0] = remote_value(((float)vx + omega*radius*cosin), ((float)vy + omega*radius*cosin));
	motor_angle[1] = remote_value(((float)vx - omega*radius*cosin), ((float)vy + omega*radius*cosin));
	motor_angle[2] = remote_value(((float)vx - omega*radius*cosin), ((float)vy - omega*radius*cosin));
	motor_angle[3] = remote_value(((float)vx + omega*radius*cosin), ((float)vy - omega*radius*cosin));
}

//float compound_movement_6020(int16_t vx,int16_t vy,int n)
//{
//	float motor_angle;
////	vx *= 300; //处理扩大遥控器所得数据
////	vy *= 300;
//	if(n==0){
//		motor_angle = remote_value(((float)vx + omega*radius*cosin), ((float)vy + omega*radius*cosin));
//	}
//	else if(n==1){
//		motor_angle = remote_value(((float)vx - omega*radius*cosin), ((float)vy + omega*radius*cosin));
//	}
//	else if(n==2){
//		motor_angle = remote_value(((float)vx - omega*radius*cosin), ((float)vy - omega*radius*cosin));
//	}
//	else if(n==3){
//		motor_angle = remote_value(((float)vx + omega*radius*cosin), ((float)vy - omega*radius*cosin));
//	}
//	return motor_angle;
//}

void rotate_6020() //仅旋转的6020角度
{
	motor_angle[0] = remote_value(omega*radius*cosin, omega*radius*cosin);
	motor_angle[1] = remote_value(- omega*radius*cosin, omega*radius*cosin);
	motor_angle[2] = remote_value(- omega*radius*cosin, - omega*radius*cosin);
	motor_angle[3] = remote_value(omega*radius*cosin, - omega*radius*cosin);
}