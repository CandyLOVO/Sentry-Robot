/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Yaw_task.h"
#include "Pitch_task.h"
#include "Launch_task.h"
#include "Exchange_task.h"
#include "imu_temp_ctrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId yawTaskHandle;
osThreadId pitchTaskHandle;
osThreadId launchTaskHandle;
osThreadId exchangeTaskHandle;
osThreadId lcdTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osThreadDef_t defaultTaskControlBlock;
osThreadId imuTempCtrlHandle;
uint32_t imuTempCtrlBuffer[ 512 ];
osThreadDef_t imuTemoCtrlControlBlock;
osSemaphoreId imuBinarySem01Handle;
osSemaphoreDef_t imuBinarySemControlBlock;
void IMU_TempCtrlTask(void * argument);

const osThreadAttr_t yawTask_attributes = {
  .name = "yawTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t pitchTask_attributes = {
  .name = "pitchTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t launchTask_attributes = {
  .name = "launchTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t exchangeTask_attributes = {
  .name = "exchangeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
const osThreadAttr_t imuTempCtrlTask_attributes = {
  .name = "imuTempCtrl",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	//osThreadNew(yawTask, Yaw_Task, osPriorityNormal, 0, 256);
	yawTaskHandle = osThreadNew(Yaw_Task, NULL, &yawTask_attributes);
	
	//osThreadDef(pitchTask, Pitch_Task, osPriorityNormal, 0, 256);
	pitchTaskHandle = osThreadNew(Pitch_Task, NULL, &pitchTask_attributes);
	
	//osThreadDef(launchTask, Launch_Task, osPriorityNormal, 0, 256);
	launchTaskHandle = osThreadNew(Launch_Task, NULL, &launchTask_attributes);
	
	//osThreadDef(exchangeTask, Exchange_Task, osPriorityRealtime, 0, 512);
	exchangeTaskHandle = osThreadNew(Exchange_Task, NULL, &exchangeTask_attributes);
	
	//osThreadDef(imuTempCtrl, IMU_TempCtrlTask, osPriorityRealtime, 0, 2048);
	imuTempCtrlHandle = osThreadNew(IMU_TempCtrlTask, NULL, &imuTempCtrlTask_attributes);
	
//	osThreadStaticDef(imuTempCtrl, IMU_TempCtrlTask, osPriorityHigh, 0, 512, imuTempCtrlBuffer, &imuTemoCtrlControlBlock);
//  imuTempCtrlHandle = osThreadCreate(osThread(imuTempCtrl), NULL);
//	osSemaphoreStaticDef(imuBinarySem01, &imuBinarySemControlBlock);
//  imuBinarySem01Handle = osSemaphoreCreate(osSemaphore(imuBinarySem01), 1);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
//		Pitch_Task();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

