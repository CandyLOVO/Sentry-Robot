#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "imuekf.h"
#include "bsp_dwt.h"

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

float gyro[3], accel[3], temp;
float gyro_sum[3];
float gyro_ave[3];
uint8_t forceStop = 0;
extern osSemaphoreId imuBinarySem01Handle;

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;
/**
************************************************************************
* @brief:      	IMU_TempCtrlTask(void const * argument)
* @param:       argument - 任务参数
* @retval:     	void
* @details:    	IMU温度控制任务函数
************************************************************************
**/
 double yaw12,pitch12,roll12;
 double INSAccelLPF;
 double INSMotionAccel_b[3];
 double INSMotionAccel_n[3];
         double a = 0.22f;
        double b = 0.78f;
        double c;
 double t_sum;
 double yaw_after = 0;
 double yaw1,pitch1,roll1;
 double yaw2,pitch2,roll2;
 double k;
 double dt01;
 double dt02;

double p[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
                                 double qt[6] = {1, 0, 0, 0, 0, 0};
                                 double dt1 = 0.002;
                                 uint32_t INS_DWT_Count = 0;
                                 double insb[3] = {0, 0, 0};

																 int i = 0;
																 int8_t flag = 0;
																 
void IMU_TempCtrlTask(void const * argument)
{
	  DWT_Init(500);
    osDelay(500);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init())
    {
        ;
    }
		for (;;)
    {
       // osSemaphoreWait(imuBinarySem01Handle, osWaitForever);
        dt1 = DWT_GetDeltaT(&INS_DWT_Count);
        BMI088_read(gyro, accel, &temp);
				err_ll = err_l;
        err_l = err;
        err = DES_TEMP - temp;
        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
        if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
        
        if (forceStop == 1)
        {
            out = 0.0f;
        }
				htim3.Instance->CCR4 = (uint16_t)out;
			
				if(flag == 0)
				{
					if(err<0.5&&err>-0.5)
					{
						gyro_sum[0] += gyro[0];
						gyro_sum[1] += gyro[1];
						gyro_sum[2] += gyro[2];
						i++;
					}
					if(i >= 2000)
					{
						gyro_ave[0] = gyro_sum[0]/2000;
						gyro_ave[1] = gyro_sum[1]/2000;
						gyro_ave[2] = gyro_sum[2]/2000;
						flag = 1;
					}
				}
				else
				{
					gyro[0] -= gyro_ave[0];
					gyro[1] -= gyro_ave[1];
					gyro[2] -= gyro_ave[2];
					imuekf((double)gyro[0], (double)gyro[1], (double)gyro[2], (double)accel[0], (double)accel[1], (double)accel[2],p,qt,dt1,&yaw12,&pitch12,&roll12,qt); 
				}
				
				osDelay(1);
    }
}
/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ACC_INT_Pin)
    {
        osSemaphoreRelease(imuBinarySem01Handle);
    }
    else if(GPIO_Pin == GYRO_INT_Pin)
    {

    }
}

//#include "main.h"
//#include "cmsis_os.h"
//#include "BMI088driver.h"
//#include "gpio.h"
//#include "tim.h"
//#include "imuekf.h"
//#include "bsp_dwt.h"
//#include "imu_temp_ctrl.h"

//#define DES_TEMP    40.0f
//#define KP          100.f
//#define KI          50.f
//#define KD          10.f
//#define MAX_OUT     500

//float gyro[3], accel[3], temp;
//uint8_t forceStop = 0;
//extern osSemaphoreId imuBinarySem01Handle;

//float out = 0;
//float err = 0;
//float err_l = 0;
//float err_ll = 0;
//float yaw_b;
///**
//************************************************************************
//* @brief:      	IMU_TempCtrlTask(void const * argument)
//* @param:       argument - 任务参数
//* @retval:     	void
//* @details:    	IMU温度控制任务函数
//************************************************************************
//**/
// double yaw12,pitch12,roll12;
// double INSAccelLPF;
// double INSMotionAccel_b[3];
// double INSMotionAccel_n[3];
//         double a = 0.22f;
//        double b = 0.78f;
//        double c;
//				int UpdateCount = 0;
//double YawAngleLast = 0;
//double YawTotalAngle = 0;
//int YawRoundCount = 0;
//float dtcuout = 0;
//extern uint8_t Rx7[100];
//#include "stm32h7xx_it.h"
//extern int u5rx;
//extern int flagrx7 ;
//extern uint8_t Rx5[78];
//  float  AccelLPF = 0.0085;
//  float  DGyroLPF = 0.009;
//	float dgyro[3];
//		extern float r1,pc1,y1;
//		int zero_c = 0;
//		float gyro_correct[3];
//		int correct_times = 0;
//double p[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
//                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
//                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
//                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
//                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
//                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
//                                 double qt[6] = {1, 0, 0, 0, 0,0};
//                                 double dt1 = 0.002;
//                                 uint32_t INS_DWT_Count = 0;
//                                 double insb[3] = {0, 0, 0};
//																 double lgyro[3];
//void IMU_TempCtrlTask(void const * argument)
//{
//	  DWT_Init(500);
//    osDelay(50);
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//    while(BMI088_init())
//    {
//        ;
//    }
//    for (;;)
//    {
//        dt1 = DWT_GetDeltaT(&INS_DWT_Count);
//			dtcuout+= dt1;
//        BMI088_read(gyro, accel, &temp);
//			        err_ll = err_l;
//        err_l = err;
//        err = DES_TEMP - temp;
//        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
//        if (out > MAX_OUT) out = MAX_OUT;
//        if (out < 0) out = 0.f;
//        
//        if (forceStop == 1)
//        {
//            out = 0.0f;
//        }
//        yaw_b = yaw12 - 0.2*dtcuout;
//        htim3.Instance->CCR4 = (uint16_t)out;

//			if(zero_c == 0)
//			{
//				if(err<0.5&&err>-0.5){
//			            gyro_correct[0]+= gyro[0];
//            gyro_correct[1]+= gyro[1];
//            gyro_correct[2]+= gyro[2];
//            correct_times++;}
//            if(correct_times>=2000)
//            {
//              gyro_correct[0]/=2000;
//              gyro_correct[1]/=2000;
//              gyro_correct[2]/=2000;
//             zero_c=2;
//            }
//			}
//				else{
//					gyro[0]-=gyro_correct[0];   //?????0?
//          gyro[1]-=gyro_correct[1];
//          gyro[2]-=gyro_correct[2];
//     		imuekf((double)gyro[0], (double)gyro[1], (double)gyro[2], (double)accel[0], (double)accel[1], (double)accel[2],p,qt,dt1,&yaw12,&pitch12,&roll12,qt); 
//				}
//				osDelay(1);
//}}

///**
//************************************************************************
//* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//* @param:       GPIO_Pin - 触发中断的GPIO引脚
//* @retval:     	void
//* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
//************************************************************************
//**/
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if(GPIO_Pin == ACC_INT_Pin)
//    {
//        osSemaphoreRelease(imuBinarySem01Handle);
//    }
//    else if(GPIO_Pin == GYRO_INT_Pin)
//    {

//    }
//}
