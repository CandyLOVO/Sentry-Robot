#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "bsp_dwt.h"
#include "imu_temp_ctrl.h"
#include "mahony_filter.h"
#include "QuaternionEKF.h"

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
float yaw_b;
/**
************************************************************************
* @brief:      	IMU_TempCtrlTask(void const * argument)
* @param:       argument - 任务参数
* @retval:     	void
* @details:    	IMU温度控制任务函数
************************************************************************
**/
 int8_t flag = 0;
 double yaw12,pitch12,roll12;
 double INSAccelLPF;
 double INSMotionAccel_b[3];
 double INSMotionAccel_n[3];
         double a = 0.22f;
        double b = 0.78f;
        double c;
				int UpdateCount = 0;
double YawAngleLast = 0;
double YawTotalAngle = 0;
int YawRoundCount = 0;
float dtcuout = 0;

//extern int flagrx7 = 0;
extern uint8_t Rx7[100];
#include "stm32h7xx_it.h"
extern int u5rx;
extern int flagrx7 ;
extern uint8_t Rx5[78];
  float  AccelLPF = 0.0085;
  float  DGyroLPF = 0.009;
	float dgyro[3];
		extern float r1,pc1,y1;
		int zero_c = 0;
		float gyro_correct[3];
		int correct_times = 0;
		float gravity[3] = {0, 0, 9.81f};

INS_t INS;
IMU_Param_t IMU_Param;
struct MAHONY_FILTER_t mahony;
		Axis3f Gyro,Accel;
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};
		void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
		
#define X 0
#define Y 1
#define Z 2

void INS_Init(void)
{ 
	 mahony_init(&mahony,1.0f,0.0f,0.001f);
   INS.AccelLPF = 0.0089f;
}
float ins_time;
int stop_time;
int count = 0;
double p[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
                                 double qt[6] = {1, 0, 0, 0, 0,0};
                                 double dt1 = 0.002;
                                 uint32_t INS_DWT_Count = 0;
                                 double insb[3] = {0, 0, 0};
																 double lgyro[3];
void IMU_TempCtrlTask(void const * argument)
{
	  DWT_Init(500);
    osDelay(50);
	  INS.AccelLPF = 0.0089f;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init())
    {
        ;
    }
		INS_Init();
    for (;;)
    {
			//
						BMI088_read(gyro, accel, &temp);
			        dt1 = DWT_GetDeltaT(&INS_DWT_Count);
			dtcuout+= dt1;
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
        yaw_b = yaw12 - 0.2*dtcuout;
        htim3.Instance->CCR4 = (uint16_t)out;
				
//			if(zero_c == 0)
//			{
//				if(err<2&&err>-2)//err<0.5&&err>-0.5
//					{
//			            gyro_correct[0]+= gyro[0];
//            gyro_correct[1]+= gyro[1];
//            gyro_correct[2]+= gyro[2];
//            correct_times++;}
//            if(correct_times>=500)
//            {
//              gyro_correct[0]/=500;
//              gyro_correct[1]/=500;
//              gyro_correct[2]/=500;
//             zero_c=2; //go to 2 state
//            }
//			
//			}
//				else{
					gyro_correct[0] = 0.00304026646;
					gyro_correct[1] = -0.00554576283;
					gyro_correct[2] = 0.00161494047;
					gyro[0]-=gyro_correct[0];   //?????0?
          gyro[1]-=gyro_correct[1];
          gyro[2]-=gyro_correct[2];
			dgyro[0] = (gyro[0] - lgyro[0])/ (DGyroLPF + dt1) + dgyro[0] * DGyroLPF / (DGyroLPF + dt1);
        dgyro[1] = (gyro[1] - lgyro[1])/ (DGyroLPF + dt1) + dgyro[1] * DGyroLPF / (DGyroLPF + dt1);
        dgyro[2] = (gyro[2] - lgyro[2])/ (DGyroLPF + dt1) + dgyro[2] * DGyroLPF / (DGyroLPF + dt1);
			lgyro[0]= gyro[0];
						lgyro[1]= gyro[1];
						lgyro[2]= gyro[2];
			if ((count % 1) == 0)
    {
        

        INS.Accel[X] = accel[X];
        INS.Accel[Y] = accel[Y];
        INS.Accel[Z] = accel[Z];
        INS.Gyro[X] = gyro[X];
        INS.Gyro[Y] = gyro[Y];
        INS.Gyro[Z] = gyro[Z];
					  Accel.x=INS.Accel[X];
	  Accel.y=INS.Accel[Y];
		Accel.z=INS.Accel[Z];

  	Gyro.x=INS.Gyro[X];
		Gyro.y=INS.Gyro[Y];
		Gyro.z=INS.Gyro[Z];
mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
	  RotationMatrix_update(&mahony);
				
		INS.q[0]=mahony.q0;
		INS.q[1]=mahony.q1;
		INS.q[2]=mahony.q2;
		INS.q[3]=mahony.q3;
       
      // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
		float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
    {
      INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt1 / (INS.AccelLPF + dt1) 
														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt1); 
//			INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt) 
//														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);			
		}
		BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n
		
		//死区处理
		if(fabsf(INS.MotionAccel_n[0])<0.02f)
		{
		  INS.MotionAccel_n[0]=0.0f;	//x轴
		}
		if(fabsf(INS.MotionAccel_n[1])<0.02f)
		{
		  INS.MotionAccel_n[1]=0.0f;	//y轴
		}
		if(fabsf(INS.MotionAccel_n[2])<0.04f)
		{
		  INS.MotionAccel_n[2]=0.0f;//z轴
		}
   		
		if(ins_time>3000.0f)
		{
			flag=1;//四元数基本收敛，加速度也基本收敛，可以开始底盘任务
			// 获取最终数据
      INS.Pitch=mahony.roll*57.3f;
		  INS.Roll=mahony.pitch*57.3f;
		  INS.Yaw=mahony.yaw*57.3f;
			yaw12 = INS.Yaw;
			pitch12 =INS.Pitch;
			roll12=INS.Roll;
		
		//INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;
		}
		else
		{
		 ins_time++;
		}
    }
//				}		
       // osSemaphoreWait(imuBinarySem01Handle, osWaitForever);
				osDelay(1);
		count++;
}}

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
/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
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
