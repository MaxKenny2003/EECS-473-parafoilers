/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "BNO085_SPI_Library.h"
#include "Hardware_Init.h"
// Declare external sensor variable (defined in main.c)
extern sensor_meta sensor1;
extern const char* get_accuracy_string(uint8_t accuracy);
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
/* Definitions for bno085Task */
osThreadId_t bno085TaskHandle;
const osThreadAttr_t bno085Task_attributes = {
  .name = "bno085Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartBno085Task(void *argument);

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

  /* creation of bno085Task */
  bno085TaskHandle = osThreadNew(StartBno085Task, NULL, &bno085Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartBno085Task */
/**
* @brief Function implementing the bno085Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBno085Task */
void StartBno085Task(void *argument)
{
  printf("BNO085 Task: Initializing...\r\n");

  // Enable both reports with delays
  printf("Enabling Game Rotation Vector...\r\n");
  enable_GameRotationVector(&sensor1, 10);
  osDelay(100);

  printf("Enabling Linear Acceleration...\r\n");
  enable_LinearAcceleration(&sensor1, 10);
  osDelay(100);

  printf("Waiting for sensor reports...\r\n");

  uint32_t timeout = HAL_GetTick() + 2000;  // 2 second timeout
  bool got_quat = false;
  bool got_accel = false;

  while (HAL_GetTick() < timeout && !(got_quat && got_accel)) {
      if(data_available(&sensor1)) {
          if (!got_quat && (sensor1.quaternions.raw_Quat_Real != 0)) {
              printf("Receiving quaternion data\r\n");
              got_quat = true;
          }

          // Check if accel data is present
          if (!got_accel && (sensor1.linear_acceleration_data.raw_Accel_X != 0 ||
                             sensor1.linear_acceleration_data.raw_Accel_Y != 0 ||
                             sensor1.linear_acceleration_data.raw_Accel_Z != 0)) {
              printf("Receiving acceleration data\r\n");
              got_accel = true;
          }
      }
      osDelay(10);
  }

  if (!got_quat) {
      printf("WARNING: Not receiving quaternion data!\r\n");
  }
  if (!got_accel) {
      printf("WARNING: Not receiving acceleration data!\r\n");
  }

  printf("BNO085 Task: Ready\r\n");

  uint32_t last_print_time = 0;

  for(;;)
  {
    if(data_available(&sensor1)) {
        float quat_i = get_Quat_I(&sensor1);
        float quat_j = get_Quat_J(&sensor1);
        float quat_k = get_Quat_K(&sensor1);
        float quat_real = get_Quat_Real(&sensor1);
        uint8_t accuracy = get_Quat_Accuracy(&sensor1);

        float accel_x = get_LinearAcceleration_X(&sensor1);
        float accel_y = get_LinearAcceleration_Y(&sensor1);
        float accel_z = get_LinearAcceleration_Z(&sensor1);

        uint32_t current_time = HAL_GetTick();
        if (current_time - last_print_time >= 100) {
            printf("Q[%.3f,%.3f,%.3f,%.3f] A[%.2f,%.2f,%.2f] %s\r\n",
                   quat_i, quat_j, quat_k, quat_real,
                   accel_x, accel_y, accel_z,
                   get_accuracy_string(accuracy));
            last_print_time = current_time;
        }
    }

    if(get_and_clear_Reset_Status(&sensor1)) {
        printf("IMU reset, re-enabling...\r\n");
        enable_GameRotationVector(&sensor1, 10);
        osDelay(100);
        enable_LinearAcceleration(&sensor1, 10);
        osDelay(100);
    }

    osDelay(5);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

