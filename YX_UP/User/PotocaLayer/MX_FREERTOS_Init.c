#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "MX_FREERTOS_Init.h"
#include "Yaw_task.h"
#include "StartDefaultTask.h"
#include "Pitch_task.h"
#include "Exchange_task.h"
#include "Friction_task.h"
#include "INS_task.h"

//================================================ÈÎÎñ´´½¨================================================//
osThreadId insTaskHandle;
osThreadId yawTaskHandle;
osThreadId defaultTaskHandle;
osThreadId pitchtaskHandle;
osThreadId exchangeHandle;
osThreadId frictionHandle;

void MX_FREERTOS_Init(void) {
  
	//²âÊÔÈÎÎñ
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	
	//YAW¿ØÖÆÈÎÎñ
	osThreadDef(yawtask, Yaw_task, osPriorityRealtime, 0, 128);		
  yawTaskHandle = osThreadCreate(osThread(yawtask), NULL);
	
	//Pitch¿ØÖÆÈÎÎñ
	osThreadDef(pitchtask, Pitch_task, osPriorityRealtime, 0, 128);
  pitchtaskHandle = osThreadCreate(osThread(pitchtask), NULL);
	
	//ÉÏÏÂC°åÍ¨ĞÅÈÎÎñ
<<<<<<< HEAD
<<<<<<< HEAD
	osThreadDef(exchangetask, Exchange_task, osPriorityIdle, 0, 512);
=======
	osThreadDef(exchangetask, Exchange_task, osPriorityIdle, 0, 256);
>>>>>>> parent of 52bf27f (ä¿®æ”¹äº†å’Œè§†è§‰çš„æ¥å£)
=======
	osThreadDef(exchangetask, Exchange_task, osPriorityRealtime, 0, 512);
>>>>>>> parent of b248010 (yawç–¯äº†ï¼Œæ•‘ä¸äº†)
  exchangeHandle = osThreadCreate(osThread(exchangetask), NULL);

	//Ä¦²ÁÂÖºÍ²¦ÅÌ¿ØÖÆÈÎÎñ
  osThreadDef(frictiontask, Friction_task, osPriorityIdle, 0, 128);
  frictionHandle = osThreadCreate(osThread(frictiontask), NULL);
	
	//ÁùÖáIMUÈÎÎñ
	osThreadDef(imutask, INS_Task,  osPriorityRealtime, 0, 1024);
  insTaskHandle = osThreadCreate(osThread(imutask), NULL);
	
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}