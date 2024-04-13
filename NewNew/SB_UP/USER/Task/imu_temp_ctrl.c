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
void IMU_TempCtrlTask(void const * argument)
{
	  DWT_Init(500);
    osDelay(500);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init())
    {
        ;
    }
//		dt01 = DWT_GetDeltaT(&INS_DWT_Count);
//		BMI088_read(gyro, accel, &temp);
//		imuekf((double)gyro[0], (double)gyro[1], (double)gyro[2], (double)accel[0], (double)accel[1], (double)accel[2],p,qt,dt01,&yaw1,&pitch1,&roll1,qt);
//		osDelay(2000);
//		dt02 = DWT_GetDeltaT(&INS_DWT_Count);
//		BMI088_read(gyro, accel, &temp);
//		imuekf((double)gyro[0], (double)gyro[1], (double)gyro[2], (double)accel[0], (double)accel[1], (double)accel[2],p,qt,dt02,&yaw2,&pitch2,&roll2,qt);
//		k = (yaw2 - yaw1)/2.0;
    for (;;)
    {
       // osSemaphoreWait(imuBinarySem01Handle, osWaitForever);
        dt1 = DWT_GetDeltaT(&INS_DWT_Count);
        BMI088_read(gyro, accel, &temp);
				t_sum += dt1;
				imuekf((double)gyro[0], (double)gyro[1], (double)gyro[2], (double)accel[0], (double)accel[1], (double)accel[2],p,qt,dt1,&yaw12,&pitch12,&roll12,qt); 
				yaw_after = yaw12 - 1.6990142*t_sum; 
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
