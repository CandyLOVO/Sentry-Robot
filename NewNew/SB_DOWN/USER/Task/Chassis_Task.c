#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "pid_user.h"
#include "can_user.h"
#include "rc_potocal.h"
#include "struct_typedef.h"
#include "math.h"
#include "Yaw_Task.h"
#include "uart_user.h"
#include "Exchange_Task.h"
#include "math.h"
#include "judge.h"
#include "stdlib.h"

/*********************************************************变量定义*********************************************************/
#define radius 3.075 // (615mm/2) m
#define cosin 0.707106781187 //二分之根号二
#define RANDOM_MAX	2500		//随机数最大值
#define RANDOM_MIN  1500  	//随机数最小值

pidTypeDef pid_3508;
pidTypeDef pid_chassis;
fp32 error_theta; //云台坐标系与底盘坐标系间夹角(此时为0~360度) 后期接收后需要对所得theta进行处理
float omega = 0; //旋转叠加计算中的角速度 rad/min
float target_speed[4]; //3508目标速度
int16_t out_speed[4]; //控制电流

extern motor_info motor[8];
extern RC_ctrl_t rc_ctrl;
extern float yaw_angle;
extern Rx_naving Rx_nav;
extern int8_t flag;
extern double yaw12; //云台陀螺仪yaw值
extern Sentry_t Sentry;
extern RTC_HandleTypeDef hrtc;

float Watch_Power_Max;
float Watch_Power;
float Watch_Buffer;
double Chassis_pidout;
double Chassis_pidout_target;
static double Scaling1=0,Scaling2=0,Scaling3=0,Scaling4=0;
float Klimit=1;
float Plimit=0;
float Chassis_pidout_max;
uint32_t random_value;
RTC_DateTypeDef  date_info;
RTC_TimeTypeDef  time_info;
int count_random = 0;
/**************************************************************************************************************************/

/*********************************************************函数定义*********************************************************/
void task_init(void);
void Yaw_Diff(void);
void chassis_calculate(int16_t x, int16_t y);
void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);
static void Chassis_Power_Limit(double Chassis_pidout_target_limit);
/**************************************************************************************************************************/

/*******************************************************底盘控制任务*******************************************************/
void Chassis_Task(void const * argument)
{
	task_init();
	
  for(;;)
  {
		if(flag == 1)
		{
		count_random++;
		if(count_random > 2000)
		{
			//生成随机数
			HAL_RTC_GetTime(&hrtc, &time_info, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &date_info, RTC_FORMAT_BIN);
			srand(time_info.Hours+time_info.Minutes+time_info.Seconds); //随机数种子设置
			random_value = rand() % (RANDOM_MAX + 1 - RANDOM_MIN) + RANDOM_MIN;//随机数生成
			count_random = 0;
		}
		
		//计算底盘与云台差角
		Yaw_Diff();
		
		//遥控器控制模式，左->中间，右->中间              自瞄调试模式，左->最下，右->中间
		if((rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==3) || (rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==2) || (rc_ctrl.rc.s[0]==2 && rc_ctrl.rc.s[1]==3))
		{
			omega = rc_ctrl.rc.ch[4]*5; //拨轮控制小陀螺
			if((rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==2) || (rc_ctrl.rc.s[0]==2 && rc_ctrl.rc.s[1]==3))
			{
				error_theta = 0;
			}
			chassis_calculate(rc_ctrl.rc.ch[0]*10, rc_ctrl.rc.ch[1]*10);
			
			for(int i=0;i<4;i++)
			{
				out_speed[i] = pid_cal_s(&pid_3508, motor[i].speed, target_speed[i]);
			}
		}
		
		//导航上场模式，左->最下，右->最下
		else if(rc_ctrl.rc.s[0]==2 && rc_ctrl.rc.s[1]==2)
		{	
		  if(Sentry.my_outpost_HP == 0) //前哨站被破
			{
				omega = 2000; //给定小陀螺转速
			}
			else
			{
				if(Sentry.my_outpost_HP != 0) //前哨战没有被破 无敌状态
				{
					omega = 0;
				}
				else
				{
					if(fabs(Rx_nav.nav_x)<=1000 && fabs(Rx_nav.nav_y)<=1000) //导航使它停下
					{
						omega = 2000; //给定小陀螺转速
					}
					else //导航发值
					{
						omega = 0;
					}
				}
			}
			
			//导航传来上坡标志位
			if(Rx_nav.poing == 1)
			{
				omega = pid_cal_a(&pid_chassis, error_theta, 0);
			}
			
//			omega = 0; //test**************************************************************************!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			
			chassis_calculate(Rx_nav.nav_x, Rx_nav.nav_y); //输入导航x、y值，CAN1传来
			for(int i=0;i<4;i++)
			{
				out_speed[i] = pid_cal_s(&pid_3508, motor[i].speed, target_speed[i]);
			}
		}
		
		//测试底盘跟随云台模式，左->最上，右->中间
		else if(rc_ctrl.rc.s[0]==3 && rc_ctrl.rc.s[1]==1)
		{
			omega = pid_cal_a(&pid_chassis, error_theta, 0);
			chassis_calculate(rc_ctrl.rc.ch[0]*15, rc_ctrl.rc.ch[1]*15); //右拨杆控制底盘
			for(int i=0;i<4;i++)
			{
				out_speed[i] = pid_cal_s(&pid_3508, motor[i].speed, target_speed[i]);
			}
		}
		
		Chassis_Power_Limit(40000);
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
	pid_init(&pid_3508, 30, 0.3, 0, 16384, 16384);
	pid_init(&pid_chassis, 2100, 0.1, 30, 16384, 16384);
	count_random = 0;
}

void Yaw_Diff()
{
	//计算底盘与云台间的相差角度
	error_theta = yaw_angle; //云台与底盘的夹角，使用5010编码值【yaw_task得到的0~180、0~-180】
	error_theta = error_theta*3.1415926/180; //转化为弧度制
//	error_theta = 0;
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
		Watch_Power_Max=Klimit;	
		Watch_Power=Sentry.Myself_chassis_power;	
		Watch_Buffer=Sentry.Myself_chassis_power_buffer;//Hero_chassis_power_buffer;//限制值，功率值，缓冲能量值，初始值是1，0，0

		Chassis_pidout_max=32768;//32768，40，960			15384 * 4，取了4个3508电机最大电流的一个保守值

		if(Watch_Power>450)
			Motor_Speed_limiting(out_speed,4096);//限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变，不知道为啥一开始用的Power>960,可以观测下这个值，看看能不能压榨缓冲功率
		else{
		Chassis_pidout=(
						fabs(out_speed[0]-motor[0].speed)+
						fabs(out_speed[1]-motor[1].speed)+
						fabs(out_speed[2]-motor[2].speed)+
						fabs(out_speed[3]-motor[3].speed));//fabs是求绝对值，这里获取了4个轮子的差值求和

		/*期望滞后占比环，增益个体加速度*/
		if(Chassis_pidout)
		{
		Scaling1=(out_speed[0]-motor[0].speed)/Chassis_pidout;	
		Scaling2=(out_speed[1]-motor[1].speed)/Chassis_pidout;
		Scaling3=(out_speed[2]-motor[2].speed)/Chassis_pidout;	
		Scaling4=(out_speed[3]-motor[3].speed)/Chassis_pidout;//求比例，4个scaling求和为1
		}
		else{Scaling1=0.25,Scaling2=0.25,Scaling3=0.25,Scaling4=0.25;}
		
		/*功率满输出占比环，车总增益加速度*/
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
		
		out_speed[0] = Scaling1*(Chassis_pidout_max*Klimit)*Plimit;//输出值
		out_speed[1] = Scaling2*(Chassis_pidout_max*Klimit)*Plimit;
		out_speed[2] = Scaling3*(Chassis_pidout_max*Klimit)*Plimit;
		out_speed[3] = Scaling4*(Chassis_pidout_max*Klimit)*Plimit;/*同比缩放电流*/
	}
}
/*************************************************************************************************************************/
