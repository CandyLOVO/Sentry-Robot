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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "struct_typedef.h"
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
#define Power1_Pin GPIO_PIN_13
#define Power1_GPIO_Port GPIOC
#define OUT_5V_Pin GPIO_PIN_14
#define OUT_5V_GPIO_Port GPIOC
#define DIR_2_Pin GPIO_PIN_15
#define DIR_2_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOC
#define Accel_INT_Pin GPIO_PIN_2
#define Accel_INT_GPIO_Port GPIOC
#define Gryo_INT_Pin GPIO_PIN_3
#define Gryo_INT_GPIO_Port GPIOC
#define Power2_Pin GPIO_PIN_4
#define Power2_GPIO_Port GPIOC
#define SPI2_Gyro_CS_Pin GPIO_PIN_0
#define SPI2_Gyro_CS_GPIO_Port GPIOB
#define SPI2_Accel_CS_Pin GPIO_PIN_1
#define SPI2_Accel_CS_GPIO_Port GPIOB
#define DIR_1_Pin GPIO_PIN_3
#define DIR_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
