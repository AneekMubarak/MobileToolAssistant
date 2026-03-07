/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "VL53L1X_api.h"
#include "VL53l1X_calibration.h"
#include "motor_hw.h"
#include "obst_det.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern I2C_HandleTypeDef hi2c1;
#define VL53_ADDR   (0x29 << 1)   // 0x52
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t distance = 0;
uint16_t count = 0;


/* USER CODE END Variables */
/* Definitions for Motor_Task */
osThreadId_t Motor_TaskHandle;
const osThreadAttr_t Motor_Task_attributes = {
  .name = "Motor_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ObstDet_Task */
osThreadId_t ObstDet_TaskHandle;
const osThreadAttr_t ObstDet_Task_attributes = {
  .name = "ObstDet_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TrackUser_Task */
osThreadId_t TrackUser_TaskHandle;
const osThreadAttr_t TrackUser_Task_attributes = {
  .name = "TrackUser_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Motor_Task_Init(void *argument);
void ObstDet_Task_Init(void *argument);
void TrackUser_Task_Init(void *argument);

extern void MX_USB_HOST_Init(void);
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
  /* creation of Motor_Task */
  Motor_TaskHandle = osThreadNew(Motor_Task_Init, NULL, &Motor_Task_attributes);

  /* creation of ObstDet_Task */
  ObstDet_TaskHandle = osThreadNew(ObstDet_Task_Init, NULL, &ObstDet_Task_attributes);

  /* creation of TrackUser_Task */
  TrackUser_TaskHandle = osThreadNew(TrackUser_Task_Init, NULL, &TrackUser_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Motor_Task_Init */
/**
  * @brief  Function implementing the Motor_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Motor_Task_Init */
void Motor_Task_Init(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN Motor_Task_Init */
  MotorHW_Init();
  const uint16_t pwm = 600;

  /* Infinite loop */
  for(;;)
  {
	  switch(dodgeAction)
	  {
	  	  case ACTION_MOVE_FORWARD:
			  Drive_Forward(pwm);
			  break;

		  case ACTION_TURN_LEFT:
			  Drive_TurnLeft(pwm);
			  break;

		  case ACTION_TURN_RIGHT:
			  Drive_TurnRight(pwm);
			  break;

		  case ACTION_STOP:
			  Drive_Stop();
			  break;
	  }

	  osDelay(20);
   }
  /* USER CODE END Motor_Task_Init */
}

/* USER CODE BEGIN Header_ObstDet_Task_Init */
/**
* @brief Function implementing the ObstDet_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ObstDet_Task_Init */
void ObstDet_Task_Init(void *argument)
{
  /* USER CODE BEGIN ObstDet_Task_Init */
	int status;

		/* Reset sensor data */
		for(int i = 0; i < TOF_SENSOR_COUNT; i++)
			ToF_ResetData(&ToF_DATA[i]);

		/* Assign I2C addresses */
		ToF_AssignAddresses();

		/* Initialize sensors */
		for(int i = 0; i < TOF_SENSOR_COUNT; i++)
			status = ToF_InitSensor(tofAddresses[i], &ToF_DATA[i]);

		for(;;)
		{
			ToF_UpdateSensors();
			ToF_SetDodgeAction();

			osDelay(20);
		}
  /* USER CODE END ObstDet_Task_Init */
}

/* USER CODE BEGIN Header_TrackUser_Task_Init */
/**
* @brief Function implementing the TrackUser_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrackUser_Task_Init */
void TrackUser_Task_Init(void *argument)
{
  /* USER CODE BEGIN TrackUser_Task_Init */
  /* Infinite loop */
  for(;;)
  {
	  //count++;
    osDelay(1000);
  }
  /* USER CODE END TrackUser_Task_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

