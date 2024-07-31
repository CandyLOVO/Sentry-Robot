/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_24V_Pin GPIO_PIN_14
#define POWER_24V_GPIO_Port GPIOC
#define POWER_5V_Pin GPIO_PIN_15
#define POWER_5V_GPIO_Port GPIOC
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOA
#define ACC_INT_Pin GPIO_PIN_10
#define ACC_INT_GPIO_Port GPIOE
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOE
#define LCD_BLK_Pin GPIO_PIN_10
#define LCD_BLK_GPIO_Port GPIOB
#define LCD_RES_Pin GPIO_PIN_11
#define LCD_RES_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_10
#define LCD_DC_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define DRIVEN_CNT 2u
#define RD 0u
#define LD 1u

#define msin(x) (sin(x))
#define mcos(x) (cos(x))
#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       // ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec
		typedef struct
{
    float q[4]; // 四元数估计值

    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数
    float DGyroLPF; // 角加速度低通滤波系数

    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;

    // IMU量测值
    float Gyro[3];  // 角速度
    float dgyro[3]; // 角加速度
    float Accel[3]; // 加速度
    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    uint8_t init;
} INS_t;

typedef struct
{
    // joint
    float phi1_w, phi4_w, phi2_w, phi5_w; // phi2_w used for calc real wheel speed
    float T_back, T_front;

    // link angle, phi1-ph5, phi5 is pod angle
    float phi1, phi2, phi3, phi4, phi5;

    // wheel
    float w_ecd;      // 电机编码器速度
    float wheel_dist; // 单侧轮子的位移
    float wheel_w;    // 单侧轮子的速度
    float body_v;     // 髋关节速度
    float T_wheel;

    // pod
    float theta, theta_w; // 杆和垂直方向的夹角,为控制状态之一
    float leg_len, legd;
    float height, height_v;
    float F_leg, T_hip;
    float target_len;

    float coord[6]; // xb yb xc yc xd yd

} LinkNPodParam;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

typedef struct
{
    // 速度
    float vel, target_v;        // 底盘速度
    float vel_m;                // 底盘速度测量值
    float vel_predict;          // 底盘速度预测值
    float vel_cov;              // 速度方差
    float acc_m, acc_last;      // 水平方向加速度,用于计算速度预测值

    // 位移
    float dist, target_dist;   // 底盘位移距离

    // IMU
    float yaw, wz, target_yaw; // yaw角度和底盘角速度
    float pitch, pitch_w;      // 底盘俯仰角度和角速度
    float roll, roll_w;        // 底盘横滚角度和角速度
    
} ChassisParam;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
