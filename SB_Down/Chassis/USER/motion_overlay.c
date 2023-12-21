#include "motion_overlay.h"
#include "handle_value.h"
#include "math.h"
#include "chassis.h"

#define cosin 0.707106781187 //二分之根号二
#define omega 20 //旋转叠加计算中的角速度
#define radius 248.248 //舵轮距离车体中心的距离 杰舵248.248mm 哨兵313.487mm
extern fp32 error_theta;

int16_t motor_angle[4]; //需要6020转到的角度
int16_t motor_speed[4];

//3508和6020各运动方式的分别控制

/**********************************************************仅旋转****************************************************************/
//仅旋转的3508速度
void rotate_3508(int16_t x,int16_t y)
{
	for(int i=0;i<4;i++){
		//将杆量反映到3508的速度
//		motor_speed[i] = sqrt(pow((float)x,2) + pow((float)y,2)); //该处理方式使得速度较小，且对角线处速度最大，前后平移时速度不大
//		motor_speed[i] = sqrt(pow((float)x,2) + pow((float)y,2))*(16384/660); //对角线处通过PID最大值限幅限制到16384
		motor_speed[i] = sqrt(pow((float)x,2) + pow((float)y,2))*(9000/660);
//		motor_speed[i] = 3000;
	}
}	

//仅旋转的6020角度
void rotate_6020()
{
//	motor_angle[0] = remote_value(omega*radius*cosin, omega*radius*cosin);
//	motor_angle[1] = remote_value(- omega*radius*cosin, omega*radius*cosin);
//	motor_angle[2] = remote_value(- omega*radius*cosin, - omega*radius*cosin);
//	motor_angle[3] = remote_value(omega*radius*cosin, - omega*radius*cosin);
	
	motor_angle[0] = remote_value(-omega*radius*cosin, omega*radius*cosin); //使得旋转和平移时3508的屁股都朝里
	motor_angle[1] = remote_value( omega*radius*cosin, omega*radius*cosin);
	motor_angle[2] = remote_value(-omega*radius*cosin, omega*radius*cosin); //3508电机左右两边两个电调方向相反（类似麦轮情况）[正常是+ - ; - -]
	motor_angle[3] = remote_value( omega*radius*cosin, omega*radius*cosin); //平移/旋转
}
/********************************************************************************************************************************/


/**********************************************************仅平移****************************************************************/
//仅平移的3508速度
void translate_3508(int16_t x,int16_t y)
{
	motor_speed[0] = sqrt(pow((float)x,2) + pow((float)y,2))*(9000/660);
	motor_speed[1] = sqrt(pow((float)x,2) + pow((float)y,2))*(9000/660);
	motor_speed[2] = -sqrt(pow((float)x,2) + pow((float)y,2))*(9000/660);
	motor_speed[3] = -sqrt(pow((float)x,2) + pow((float)y,2))*(9000/660);
}	

//仅平移的6020角度
void translate_6020(int16_t x,int16_t y)
{
	int16_t vx = x*cos(error_theta) - y*sin(error_theta);
	int16_t vy = x*sin(error_theta) + y*cos(error_theta);
	for(int i=0;i<4;i++){
		//将遥控器希望转到的角度投影 0~180/0~-180
		motor_angle[i] = remote_value((float)vx , (float)vy);
	}
}
/********************************************************************************************************************************/


/***********************************************************旋转+平移************************************************************/
//旋转+平移的3508速度
void compound_movement_3508(int16_t x,int16_t y)
{
	int16_t vx = (x*cos(error_theta) - y*sin(error_theta))*(3000/660); //底盘跟随云台处理后的x、y方向的速度
	int16_t vy = (x*sin(error_theta) + y*cos(error_theta))*(3000/660);
//	motor_speed[0] = sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2)); //平移与旋转叠加后的向量长度
//	motor_speed[1] = sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2));
//	motor_speed[2] = sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
//	motor_speed[3] = sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
	
	motor_speed[0] = sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2)); //平移与旋转叠加后的向量长度
	motor_speed[1] = sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy + omega*radius*cosin),2));
	motor_speed[2] = sqrt(pow(((float)vx + omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
	motor_speed[3] = sqrt(pow(((float)vx - omega*radius*cosin),2) + pow(((float)vy - omega*radius*cosin),2));
}

//旋转+平移的6020角度
void compound_movement_6020(int16_t x,int16_t y)
{
	int16_t vx = (x*cos(error_theta) - y*sin(error_theta))*(3000/660);
	int16_t vy = (x*sin(error_theta) + y*cos(error_theta))*(3000/660);
//	motor_angle[0] = remote_value(((float)vx + omega*radius*cosin), ((float)vy + omega*radius*cosin));
//	motor_angle[1] = remote_value(((float)vx - omega*radius*cosin), ((float)vy + omega*radius*cosin));
//	motor_angle[2] = remote_value(((float)vx - omega*radius*cosin), ((float)vy - omega*radius*cosin));
//	motor_angle[3] = remote_value(((float)vx + omega*radius*cosin), ((float)vy - omega*radius*cosin));
	
	motor_angle[0] = remote_value(((float)vx - omega*radius*cosin), ((float)vy + omega*radius*cosin));
	motor_angle[1] = remote_value(((float)vx + omega*radius*cosin), ((float)vy + omega*radius*cosin));
	motor_angle[2] = remote_value(((float)vx - omega*radius*cosin), ((float)vy + omega*radius*cosin));
	motor_angle[3] = remote_value(((float)vx + omega*radius*cosin), ((float)vy + omega*radius*cosin));
}
/********************************************************************************************************************************/

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