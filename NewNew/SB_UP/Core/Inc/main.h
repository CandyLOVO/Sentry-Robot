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

#define RPM_2_ANGLE_PER_SEC 6.0f       // ��360��/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f // ��2pi/60sec
		typedef struct
{
    float q[4]; // ��Ԫ������ֵ

    float MotionAccel_b[3]; // ����������ٶ�
    float MotionAccel_n[3]; // ����ϵ���ٶ�

    float AccelLPF; // ���ٶȵ�ͨ�˲�ϵ��
    float DGyroLPF; // �Ǽ��ٶȵ�ͨ�˲�ϵ��

    // bodyframe�ھ���ϵ��������ʾ
    float xn[3];
    float yn[3];
    float zn[3];

    // ���ٶ��ڻ���ϵ��XY����ļн�
    // float atanxz;
    // float atanyz;

    // IMU����ֵ
    float Gyro[3];  // ���ٶ�
    float dgyro[3]; // �Ǽ��ٶ�
    float Accel[3]; // ���ٶ�
    // λ��
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
    float w_ecd;      // ����������ٶ�
    float wheel_dist; // �������ӵ�λ��
    float wheel_w;    // �������ӵ��ٶ�
    float body_v;     // �Źؽ��ٶ�
    float T_wheel;

    // pod
    float theta, theta_w; // �˺ʹ�ֱ����ļн�,Ϊ����״̬֮һ
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
    // �ٶ�
    float vel, target_v;        // �����ٶ�
    float vel_m;                // �����ٶȲ���ֵ
    float vel_predict;          // �����ٶ�Ԥ��ֵ
    float vel_cov;              // �ٶȷ���
    float acc_m, acc_last;      // ˮƽ������ٶ�,���ڼ����ٶ�Ԥ��ֵ

    // λ��
    float dist, target_dist;   // ����λ�ƾ���

    // IMU
    float yaw, wz, target_yaw; // yaw�ǶȺ͵��̽��ٶ�
    float pitch, pitch_w;      // ���̸����ǶȺͽ��ٶ�
    float roll, roll_w;        // ���̺���ǶȺͽ��ٶ�
    
} ChassisParam;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
