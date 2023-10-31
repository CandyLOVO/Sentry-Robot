#include "motion_overlay.h"
#include "handle_value.h"
#include "math.h"

#define cosin 0.717 //二分之根号二
#define omega 3000 //旋转叠加计算中的角速度
#define radius 500 //舵轮距离车体中心的距离

float* compound_movement_3508(int16_t vx,int16_t vy) //使用：float* b; b = compound_movement_3508(); b为数组
{
	static float motor_value[4];
	motor_value[0] = sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2));
	motor_value[1] = sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
	motor_value[2] = sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
	motor_value[3] = sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2));
	return motor_value;
}

float* compound_movement_6020(int16_t vx,int16_t vy)
{
	static float motor_angle[4];
	motor_angle[0] = remote_value(((float)vx + omega*radius*cosin), ((float)vy + omega*radius*cosin));
	motor_angle[1] = remote_value(((float)vx + omega*radius*cosin), ((float)vy - omega*radius*cosin));
	motor_angle[2] = remote_value(((float)vx - omega*radius*cosin), ((float)vy - omega*radius*cosin));
	motor_angle[3] = remote_value(((float)vx - omega*radius*cosin), ((float)vy + omega*radius*cosin));
	return motor_angle;
}