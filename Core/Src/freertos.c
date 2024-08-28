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
#include "CubeMars.h"
#include "TimeSharing.h"
#include "Braking_System.h"
#include "Lifter_M01.h"
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
extern uint8_t TransmissionDataToMotorsErrorFlag;

/* USER CODE END Variables */
/* Definitions for BackendTask */
osThreadId_t BackendTaskHandle;
const osThreadAttr_t BackendTask_attributes = {
  .name = "BackendTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Application */
osThreadId_t ApplicationHandle;
const osThreadAttr_t Application_attributes = {
  .name = "Application",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Backend_Task(void *argument);
void Application_Task(void *argument);

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
  /* creation of BackendTask */
  BackendTaskHandle = osThreadNew(Backend_Task, NULL, &BackendTask_attributes);

  /* creation of Application */
  ApplicationHandle = osThreadNew(Application_Task, NULL, &Application_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Backend_Task */
/**
  * @brief  Function implementing the BackendTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Backend_Task */
void Backend_Task(void *argument)
{
  /* USER CODE BEGIN Backend_Task */
  /* Infinite loop */
  HAL_Delay(100);
  for(;;)
  {
	/*Handle motor messaging stream tasks*/
	for (uint8_t tid = 1;tid <= Number_of_Motor; tid++)
	{
		if 	(Time_Sharing_Handle(0,tid, 10))
		{
			if (sys_ok == CubeMars_bulid_CAN_command(&MotorControl[tid-1])) {
				if(sys_ok != CAN_TransmitToBrakeMotors())
					TransmissionDataToMotorsErrorFlag = 1;
			}
		}
	}
//	if 	(Time_Sharing_Handle(1,1, 10))
//		SendBrakeStatusToMainBoard();
//	if (Time_Sharing_Handle(1,2, 10))
//		SendBrakeValuesToMainBoard();
//	if (Time_Sharing_Handle(1,3, 10))
//		SendMotorTemperatureToMainBoard();
//	if (Time_Sharing_Handle(1,4, 10))
//		SendMotorCurrentToMainBoard();
    osDelay(1);
  }
  /* USER CODE END Backend_Task */
}

/* USER CODE BEGIN Header_Application_Task */
/**
* @brief Function implementing the Application thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Application_Task */
void Application_Task(void *argument)
{
  /* USER CODE BEGIN Application_Task */
	 Braking_System_Config();
  /* Infinite loop */
  for(;;)
  {
#if(SYSTEM_M01 == SYSTEM_M01_LIFTER_TEST)
	  Lifter_M01_Loop();

#elif(SYSTEM_M01 == SYSTEM_M01_BRAKING_TEST)

	  Braking_System_Loop_M01();
#endif

#if(SYSTEM_M01 == SYSTEM_M01_LIFTER_TEST)
	  osDelay(LIFTER_TASK_CYCLE_TIME);

#else
	  osDelay(1);
#endif

  }
  /* USER CODE END Application_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

