#include "imu_temp_control_task.h"
#include "BMI088driver.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid_imu.h"
#include "bsp_imu_pwm.h"
#include "imuekf.h"
#include "bsp_dwt.h"
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwmï¿½ï¿½ï¿½ï¿½

#define TEMPERATURE_PID_KP 800.0f //kp of temperature control PID 
#define TEMPERATURE_PID_KI 0.02f    //ki of temperature control PID 
#define TEMPERATURE_PID_KD 5.0f    //kd of temperature control PID 

#define TEMPERATURE_PID_MAX_OUT 600.0f  //max out of temperature control PID 
#define TEMPERATURE_PID_MAX_IOUT 600.0f //max iout of temperature control PID 


extern SPI_HandleTypeDef hspi2;
extern float Yaw_middle_c;	//Ò»¼¶ÔÆÌ¨yaw(Ö»ÓÐ¾ø¶Ô×ø±ê) 9025×ª»¯Îª0~+-180ºóµÄ±àÂëÖµ

//task handler ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
TaskHandle_t INS_task_local_handler;


volatile uint8_t imu_start_flag = 0;

fp32 gyro[3], accel[3], temp;

//kp, ki,kd three params
const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
//pid struct 
pid_type_def imu_temp_pid;
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
  * @brief          bmi088 temperature control 
  * @param[in]      argument: NULL
  * @retval         none
  */
/**
  * @brief          bmi088ï¿½Â¶È¿ï¿½ï¿½ï¿½
  * @param[in]      argument: NULL
  * @retval         none
  */
 double yaw12,pitch12,roll12;
 double INSAccelLPF;
 double INSMotionAccel_b[3];
 double INSMotionAccel_n[3];
         double a = 0.22f;
        double b = 0.78f;
        double c;

double p[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 10000, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 10000};
                                 double qt[6] = {1, 0, 0, 0, 0.1, 0.1};
                                 double dt1 = 0.002;
                                 uint32_t INS_DWT_Count = 0;
                                 double insb[3] = {0, 0, 0};
void imu_temp_control_task()
{
    osDelay(500);
    //pid init  PIDï¿½ï¿½Ê¼ï¿½ï¿½
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    
   
    //bmi088 init. bmi088ï¿½ï¿½Ê¼ï¿½ï¿½

    //set spi frequency
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
    //get task handle, must enable 'xTaskGetHandle' in cubeMX
    //ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½cubeMXÊ¹ï¿½ï¿½'xTaskGetHandle'
 
    imu_start_flag = 1;
    BMI088_read(gyro, accel, &temp);
  const float gravity[3] = {0, 0, 9.78f};
    while(1)
    {
        //wait for task waked up
        //ï¿½È´ï¿½ï¿½ï¿½ï¿½ñ±»»ï¿½ï¿½ï¿½
       
        //read data.ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½
         dt1 = DWT_GetDeltaT(&INS_DWT_Count);
        BMI088_read(gyro, accel, &temp);
        

        

        uint16_t tempPWM;
        //pid calculate. PIDï¿½ï¿½ï¿½ï¿½
        PID_calc(&imu_temp_pid, temp, 40.0f);
        
if (p[28] > 10000)
    {
        p[28] = 10000;
    }
    if (p[35] > 10000)
    {
        p[35] = 10000;
    }
        imuekf((double)gyro[0], (double)gyro[1], (double)gyro[2], (double)accel[0], (double)accel[1], (double)accel[2],p,qt,dt1,&yaw12,&pitch12,&roll12,qt); 
 
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
				
				Yaw_middle_c = yaw12;
				
        osDelay(1);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == Accel_INT_Pin)
    {

        if(imu_start_flag)
        {
            //wake up the task
            //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
            {
                static BaseType_t xHigherPriorityTaskWoken;
                vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
    else if(GPIO_Pin == Gryo_INT_Pin)
    {

    }
}
