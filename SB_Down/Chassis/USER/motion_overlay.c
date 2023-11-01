#include "motion_overlay.h"
#include "handle_value.h"
#include "math.h"

#define cosin 0.707106781187 //二分之根号二
#define omega 13 //旋转叠加计算中的角速度
#define radius 50 //舵轮距离车体中心的距离

int16_t motor_angle[4];
int16_t motor_speed[4];

void compound_movement_3508(int16_t vx,int16_t vy) 
{
	motor_speed[0] = sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2));
	motor_speed[1] = sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2));
	motor_speed[2] = sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
	motor_speed[3] = sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
}

void compound_movement_6020(int16_t vx,int16_t vy)
{
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