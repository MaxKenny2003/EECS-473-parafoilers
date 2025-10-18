/* freertos.c - PATCHED VERSION WITH FIXES */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "Servo.h"
#include "Xbee.h"
#include "NEO_M9_UART.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

osMutexId_t printfMutexHandle;
const osMutexAttr_t printfMutex_attributes = {
  .name = "printfMutex"
};

// Task Handles and Attributes
osThreadId_t readGPSHandle;
const osThreadAttr_t readGPS_attributes = {
  .name = "readGPS",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t UpdateServosHandle;
const osThreadAttr_t UpdateServos_attributes = {
  .name = "UpdateServos",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t ReadCoordHandle;
const osThreadAttr_t ReadCoord_attributes = {
  .name = "ReadCoord",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t bno085TaskHandle;
const osThreadAttr_t bno085Task_attributes = {
  .name = "bno085Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal, // Lowered priority
};

// Semaphore Handle
osSemaphoreId_t gpsDataReadySemHandle;
const osSemaphoreAttr_t gpsDataReadySem_attributes = {
  .name = "gpsDataReadySem"
};

//Sempahore Handle IMU
osSemaphoreId_t imuDataReadySemHandle;
const osSemaphoreAttr_t imuDataReadySem_attributes = {
  .name = "imuDataReadySem"
};

// External hardware handles and variables needed by the tasks

volatile control_mode_t control_mode = MODE_GPS_AUTO;
volatile uint8_t manual_servo_command = 0;
volatile uint32_t last_manual_command_time = 0;

extern UART_HandleTypeDef huart1;
extern GPS_DATA_t myGpsData;
extern GPS_Info targetLocation;
extern volatile uint8_t new_coord_flag;
extern uint8_t NEW_COORD_BUF[8];
extern volatile uint8_t gpsRxBuffer[];
extern volatile uint16_t gps_data_length;
extern sensor_meta sensor1;


// Variable updates by GPS taskk every time it receives new data
volatile uint32_t lastGpsUpdateTick = 0;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
void read_GPS(void *argument);
void updateServos(void *argument);
void readCoord(void *argument);
void StartBno085Task(void *argument);
const char* get_accuracy_string(uint8_t accuracy);
void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void);

/**
  * @brief  FreeRTOS initialization
  */
void MX_FREERTOS_Init(void) {
  printfMutexHandle = osMutexNew(&printfMutex_attributes);
  if (printfMutexHandle == NULL) {
      printf("ERROR: printfMutex creation failed\r\n");
      Error_Handler();
  }

  gpsDataReadySemHandle = osSemaphoreNew(1, 0, &gpsDataReadySem_attributes);
  if (gpsDataReadySemHandle == NULL) {
      printf("ERROR: gpsDataReadySem creation failed\r\n");
      Error_Handler();
  }

  imuDataReadySemHandle = osSemaphoreNew(1, 0, &imuDataReadySem_attributes);
  if (imuDataReadySemHandle == NULL) {
      printf("ERROR: imuDataReadySem creation failed\r\n");
      Error_Handler();
  }
  printf("Semaphore/mutex created successfully\r\n");

  printf("Creating defaultTask...\r\n");
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  if (defaultTaskHandle == NULL) { Error_Handler(); }
  printf("defaultTask created\r\n");

  printf("Creating readGPS task...\r\n");
  readGPSHandle = osThreadNew(read_GPS, NULL, &readGPS_attributes);
  if (readGPSHandle == NULL) { Error_Handler(); }
  printf("readGPS task created\r\n");

  printf("Creating UpdateServos task...\r\n");
  UpdateServosHandle = osThreadNew(updateServos, NULL, &UpdateServos_attributes);
  if (UpdateServosHandle == NULL) { Error_Handler(); }
  printf("UpdateServos task created\r\n");

  printf("Creating ReadCoord task...\r\n");
  ReadCoordHandle = osThreadNew(readCoord, NULL, &ReadCoord_attributes);
  if (ReadCoordHandle == NULL) { Error_Handler(); }
  printf("ReadCoord task created\r\n");

  printf("Creating bno085Task...\r\n");
  bno085TaskHandle = osThreadNew(StartBno085Task, NULL, &bno085Task_attributes);
  if (bno085TaskHandle == NULL) { Error_Handler(); }
  printf("bno085Task created\r\n");
}

void StartDefaultTask(void *argument)
{
  for(;;) { osDelay(1000); }
}

void read_GPS(void *argument)
{
    uint8_t local_gps_buffer[RX_BUFFER_LEN];
    uint16_t local_data_length;

    osMutexAcquire(printfMutexHandle, osWaitForever);
    printf("GPS Task Started\r\n");
    osMutexRelease(printfMutexHandle);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)gpsRxBuffer, RX_BUFFER_LEN);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    osMutexAcquire(printfMutexHandle, osWaitForever);
    printf("GPS DMA reception started\r\n");
    osMutexRelease(printfMutexHandle);

    for(;;)
    {
        if (osSemaphoreAcquire(gpsDataReadySemHandle, osWaitForever) == osOK)
        {
        	//enter area where GPS data cannot be interrupted as we get its data
            taskENTER_CRITICAL();
            memcpy(local_gps_buffer, (void*)gpsRxBuffer, gps_data_length);
            local_data_length = gps_data_length;
            taskEXIT_CRITICAL();

            processGPS(local_gps_buffer, local_data_length);

            if(myGpsData.newData) {
                myGpsData.newData = 0;
                //update gps time stamp
                lastGpsUpdateTick = osKernelGetTickCount();

                osMutexAcquire(printfMutexHandle, osWaitForever);
                printf("GPS: %.6f, %.6f, %.1fm, Sats=%d\r\n",
                       myGpsData.Latitude, myGpsData.Longitude,
                       myGpsData.Altitude, myGpsData.Satellites);
                osMutexRelease(printfMutexHandle);
            }
        }
    }
}

void updateServos(void *argument)
{
  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("UpdateServos Task Started.\r\n");
  osMutexRelease(printfMutexHandle);

  // Wait for GPS fix or manual mode
  while (myGpsData.Fix_Type == 0 && control_mode == MODE_GPS_AUTO) {
      osMutexAcquire(printfMutexHandle, osWaitForever);
      printf("Servo Task: Waiting for GPS fix or manual control. Motor stopped.\r\n");
      osMutexRelease(printfMutexHandle);

      stop_servo();
      osDelay(1000);
  }

  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("Servo Task: Control active.\r\n");
  osMutexRelease(printfMutexHandle);

  for(;;)
  {
	    // Auto-exit FALLBACK manual mode when GPS+target is ready
	    if(control_mode == MODE_MANUAL_FALLBACK) {
	        bool has_valid_fix = (myGpsData.Fix_Type > 0);
	        bool has_target = (targetLocation.latitude != 0);
	        uint32_t current_tick = osKernelGetTickCount();
	        bool gps_is_fresh = (lastGpsUpdateTick > 0) && ((current_tick - lastGpsUpdateTick) < 2000);

	        if(has_valid_fix && has_target && gps_is_fresh) {
	            osMutexAcquire(printfMutexHandle, osWaitForever);
	            printf("Auto-switching from fallback to GPS control mode\r\n");
	            osMutexRelease(printfMutexHandle);
	            control_mode = MODE_GPS_AUTO;
	            manual_servo_command = 0;
	        }
	    }

	    // Execute control based on mode
	    if(control_mode == MODE_MANUAL_FALLBACK || control_mode == MODE_MANUAL_OVERRIDE) {
	        // Manual control (either fallback or override)
	        if(manual_servo_command == 1) {
	            spin_servo(345);  // Left
	        } else if(manual_servo_command == 2) {
	            spin_servo(230);  // Right
	        } else {
	            stop_servo();
	        }
    }
    else {
        // Attempt at automatic gps-based control (needs logic to allow override and stop gps from retaking control
        uint32_t current_tick = osKernelGetTickCount();
        if (lastGpsUpdateTick == 0) {
            lastGpsUpdateTick = current_tick;
        }
        bool gps_signal_lost = (current_tick - lastGpsUpdateTick) > 2000;
        bool has_valid_fix = (myGpsData.Fix_Type > 0);

        if(has_valid_fix && targetLocation.latitude != 0 && !gps_signal_lost)
        {
            float difference = myGpsData.Latitude - targetLocation.latitude;
            float threshold = 0.0001;

            if(difference > threshold) {
                spin_servo_clockwise();
            } else if(difference < -threshold){
                spin_servo_counterclockwise();
            } else {
                stop_servo();
            }
        } else {
            stop_servo();
        }
    }
    osDelay(200);
  }
}

void readCoord(void *argument)
{
  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("ReadCoord Task Started.\r\n");
  osMutexRelease(printfMutexHandle);

  for(;;)
  {
    if(new_coord_flag) {
        new_coord_flag = 0;

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

        int32_t lat = (NEW_COORD_BUF[0] << 24) | (NEW_COORD_BUF[1] << 16) |
                      (NEW_COORD_BUF[2] << 8) | NEW_COORD_BUF[3];
        int32_t lon = (NEW_COORD_BUF[4] << 24) | (NEW_COORD_BUF[5] << 16) |
                      (NEW_COORD_BUF[6] << 8) | NEW_COORD_BUF[7];

        float lat_float = (float)lat / 10000000.0f;
        float lon_float = (float)lon / 10000000.0f;

        targetLocation.latitude = lat_float;
        targetLocation.longitude = lon_float;

        osMutexAcquire(printfMutexHandle, osWaitForever);
        printf("New coordinate received from Xbee!\r\n");
        printf("  Target Latitude:  %.6f\r\n", targetLocation.latitude);
        printf("  Target Longitude: %.6f\r\n", targetLocation.longitude);
        osMutexRelease(printfMutexHandle);
    }
    osDelay(100);
  }
}

void StartBno085Task(void *argument)
{
  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("BNO085 Task: Initializing...\r\n");
  osMutexRelease(printfMutexHandle);

  enable_Gyroscope(&sensor1, 10);
  osDelay(250);
  enable_LinearAcceleration(&sensor1, 10);
  osDelay(250);
  enable_GameRotationVector(&sensor1, 10);
  osDelay(250);

  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("BNO085 Task: Waiting for sensor to stabilize...\r\n");
  osMutexRelease(printfMutexHandle);
  osDelay(500);

  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("BNO085 Task: Taring sensor...\r\n");
  osMutexRelease(printfMutexHandle);
  tare_now(&sensor1, TARE_AXES_ALL, TARE_BASIS_GAME_ROT_VEC);
  osDelay(100);

  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("BNO085 Task: Ready, interrupt driven\r\n");
  osMutexRelease(printfMutexHandle);

  uint32_t last_print_time = 0;
  uint32_t interrupt_count = 0;
  uint32_t data_read_count = 0;
  uint32_t last_stats_time = 0;

  for(;;)
  {
    // Check for reset (still polled, happens rarely)
    if(get_and_clear_Reset_Status(&sensor1)) {
        osMutexAcquire(printfMutexHandle, osWaitForever);
        printf("IMU reset detected, re-enabling reports...\r\n");
        osMutexRelease(printfMutexHandle);
        enable_Gyroscope(&sensor1, 10);
        osDelay(250);
        enable_LinearAcceleration(&sensor1, 10);
        osDelay(250);
        enable_GameRotationVector(&sensor1, 10);
        osDelay(250);
        tare_now(&sensor1, TARE_AXES_ALL, TARE_BASIS_GAME_ROT_VEC);
        osDelay(500);
    }

    // Wait for interrupt signal (with 100ms timeout as safety)
    if(osSemaphoreAcquire(imuDataReadySemHandle, 100) == osOK) {
        interrupt_count++;

        if(data_available(&sensor1)) {
            data_read_count++;

            float quat_i = get_Quat_I(&sensor1);
            float quat_j = get_Quat_J(&sensor1);
            float quat_k = get_Quat_K(&sensor1);
            float quat_real = get_Quat_Real(&sensor1);
            uint8_t accuracy = get_Quat_Accuracy(&sensor1);

            float accel_x = get_LinearAcceleration_X(&sensor1);
            float accel_y = get_LinearAcceleration_Y(&sensor1);
            float accel_z = get_LinearAcceleration_Z(&sensor1);

            float gyro_x = get_Gyroscope_X(&sensor1);
            float gyro_y = get_Gyroscope_Y(&sensor1);
            float gyro_z = get_Gyroscope_Z(&sensor1);

            uint32_t current_time = osKernelGetTickCount();
            if (current_time - last_print_time >= 500) {
                if(osMutexAcquire(printfMutexHandle, 10) == osOK) {
                    printf("Q[%.3f,%.3f,%.3f,%.3f] A[%.2f,%.2f,%.2f] G[%.3f,%.3f,%.3f] %s\r\n",
                           quat_i, quat_j, quat_k, quat_real,
                           accel_x, accel_y, accel_z,
                           gyro_x, gyro_y, gyro_z,
                           get_accuracy_string(accuracy));
                    osMutexRelease(printfMutexHandle);
                }
                last_print_time = current_time;
            }
        }
    } else {
        // Timeout - no interrupt received
        // This is normal during initial startup or if sensor stops
    }

    // Small delay to check for resets periodically
    // Not strictly necessary since we're interrupt-driven now
    osDelay(10);
  }
}


const char* get_accuracy_string(uint8_t accuracy) {
    switch (accuracy) {
        case 0: return "Unreliable";
        case 1: return "Low";
        case 2: return "Medium";
        case 3: return "High";
        default: return "Unknown";
    }
}

