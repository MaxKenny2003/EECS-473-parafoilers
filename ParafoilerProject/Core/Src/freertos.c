/* freertos.c - */

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
#include "Control.h"

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
  .priority = (osPriority_t) osPriorityNormal
};

//osThreadId_t MoveServosHandle;
//const osThreadAttr_t MoveServos_attributes = {
//  .name = "moveServos",
//  .stack_size = 512 * 2,
//  .priority = (osPriority_t) osPriorityHigh2,
//};

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

osThreadId_t UpdateHeadingOffsetHandle;
const osThreadAttr_t UpdateHeadingOffset_attributes = {
  .name = "UpdateHeadingOffset",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t SendDataHandle;
const osThreadAttr_t SendData_attributes = {
  .name = "SendData",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityLow,
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

osMutexId_t navDataMutexHandle;
const osMutexAttr_t navDataMutex_attributes = {
  .name = "navDataMutex"
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
extern Servo_Instance_t ServoL; // Use the global instance
extern Servo_Instance_t ServoR;

//volatile float gps_cog = 0;

typedef struct {
    float imu_heading;      // 0-360 from IMU
    float heading_offset;   //
    float heading_real;     // 0-360 corrected heading
    float target_angle;     // 0-360 desired heading
    uint32_t lastGpsUpdateTick;
} NavData_t;

volatile NavData_t navData = {0};

volatile int servo_count = 0;

volatile float global_accel_x = 0, global_accel_y  = 0, global_accel_z = 0;
volatile float global_gyro_x = 0, global_gyro_y  = 0, global_gyro_z = 0;
volatile float global_quat_i = 0, global_quat_j = 0, global_quat_k = 0, global_quat_real = 0;


// Variable updates by GPS taskk every time it receives new data
// volatile uint32_t lastGpsUpdateTick = 0; // Moved into NavData struct

/* Private function prototypes -----------------------------------------------*/
void read_GPS(void *argument);
void updateServos(void *argument);
//void moveServos(void *argument);
void readCoord(void *argument);
void StartBno085Task(void *argument);
void updateHeadingOffset(void *argument);
void sendData(void *argument);
const char* get_accuracy_string(uint8_t accuracy);

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
  navDataMutexHandle = osMutexNew(&navDataMutex_attributes);
  if (navDataMutexHandle == NULL) {
      printf("ERROR: navDataMutex creation failed\r\n");
      Error_Handler();
  }

  printf("Semaphore/mutex created successfully\r\n");


  printf("Creating readGPS task...\r\n");
  readGPSHandle = osThreadNew(read_GPS, NULL, &readGPS_attributes);
  if (readGPSHandle == NULL) { Error_Handler(); }
  printf("readGPS task created\r\n");

  printf("Creating UpdateServos task...\r\n");
  UpdateServosHandle = osThreadNew(updateServos, NULL, &UpdateServos_attributes);
  if (UpdateServosHandle == NULL) { Error_Handler(); }
  printf("UpdateServos task created\r\n");

//  printf("Creating MoveServos task...\r\n");
//  MoveServosHandle = osThreadNew(moveServos, NULL, &MoveServos_attributes);
//  if (MoveServosHandle == NULL) { Error_Handler(); }
//  printf("MoveServos task created\r\n");


  printf("Creating ReadCoord task...\r\n");
  ReadCoordHandle = osThreadNew(readCoord, NULL, &ReadCoord_attributes);
  if (ReadCoordHandle == NULL) { Error_Handler(); }
  printf("ReadCoord task created\r\n");

  printf("Creating bno085Task...\r\n");
  bno085TaskHandle = osThreadNew(StartBno085Task, NULL, &bno085Task_attributes);
  if (bno085TaskHandle == NULL) { Error_Handler(); }
  printf("bno085Task created\r\n");

  printf("Creating UpdateHeadingOffset task...\r\n");
  UpdateHeadingOffsetHandle = osThreadNew(updateHeadingOffset, NULL, &UpdateHeadingOffset_attributes);
  if (UpdateHeadingOffsetHandle == NULL) { Error_Handler(); }
  printf("UpdateHeadingOffset created\r\n");

  printf("Creating SendData task...\r\n");
  SendDataHandle = osThreadNew(sendData, NULL, &SendData_attributes);
  if (SendDataHandle == NULL) { Error_Handler(); }
  printf("SendData created\r\n");
}



void read_GPS(void *argument)
{
    uint8_t local_gps_buffer[RX_BUFFER_LEN];
    uint16_t local_data_length;
    float new_target_angle; // Local variable

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

                uint32_t current_tick = osKernelGetTickCount();

                /*
                osMutexAcquire(printfMutexHandle, osWaitForever);
                printf("GPS: %.6f, %.6f, %.1fm, Sats=%d, %.6f, %.6f\r\n",
                       myGpsData.Latitude, myGpsData.Longitude,
                       myGpsData.Altitude, myGpsData.Satellites, myGpsData.headMot, myGpsData.headAcc);
                osMutexRelease(printfMutexHandle);
                */
                // Print a safer version
                osMutexAcquire(printfMutexHandle, osWaitForever);
                printf("GPS: Sats=%d\r\n", myGpsData.Satellites);
                osMutexRelease(printfMutexHandle);


                // Need to read targetLocation safely
                float local_target_lon, local_target_lat;
                if (osMutexAcquire(navDataMutexHandle, 100) == osOK) {
                    local_target_lon = targetLocation.longitude;
                    local_target_lat = targetLocation.latitude;
                    osMutexRelease(navDataMutexHandle);
                } else {
                    // Failed to get mutex, skip this calculation
                    continue;
                }

                // Calculate the new target angle locally
                new_target_angle = calc_angle(local_target_lon, local_target_lat, myGpsData.Longitude, myGpsData.Latitude);

                // Use mutex to safely update shared nav data
                if (osMutexAcquire(navDataMutexHandle, 100) == osOK) {
                    navData.target_angle = new_target_angle;
                    navData.lastGpsUpdateTick = current_tick;
                    osMutexRelease(navDataMutexHandle);
                }

                osMutexAcquire(printfMutexHandle, osWaitForever);
                printf("Angle to turn (int): %d\r\n", (int)new_target_angle);
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

  // Local copies of shared variables
  float local_target_angle;
  float local_heading_real;
  uint32_t local_lastGpsUpdateTick;

  // Wait for GPS fix or manual mode
  while (myGpsData.Fix_Type == 0 && control_mode == MODE_GPS_AUTO) {
      osMutexAcquire(printfMutexHandle, osWaitForever);
      printf("Servo Task: Waiting for GPS fix or manual control. Motor stopped.\r\n");
      osMutexRelease(printfMutexHandle);

      stop_servo(&ServoL);
      stop_servo(&ServoR);
      osDelay(1000);
  }

  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("Servo Task: Control active.\r\n");
  osMutexRelease(printfMutexHandle);

  // ServoL.last_update_tick = osKernelGetTickCount(); // This is set in servo_init
  // ServoR.last_update_tick = osKernelGetTickCount();

  for(;;)
  {
        uint32_t current_tick = osKernelGetTickCount();

        // Safely read the shared variable using mutex
        if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
            local_lastGpsUpdateTick = navData.lastGpsUpdateTick;
            osMutexRelease(navDataMutexHandle);
        } else {
            // Failed to get mutex, use stale data for this loop
        }


	    // Auto-exit FALLBACK manual mode when GPS+target is ready
	    if(control_mode == MODE_MANUAL_FALLBACK) {
            // Need to read targetLocation safely
            float local_target_lat = 0;
            if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
                local_target_lat = targetLocation.latitude;
                osMutexRelease(navDataMutexHandle);
            }

	        bool has_valid_fix = (myGpsData.Fix_Type > 0);
	        bool has_target = (local_target_lat != 0);

	        bool gps_is_fresh = (local_lastGpsUpdateTick > 0) && ((current_tick - local_lastGpsUpdateTick) < 2000);

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
	            spin_servo(&ServoL, SERVO_FORWARD_FAST);  // Left (Pull)
                spin_servo(&ServoR, SERVO_REVERSE_FAST); // Right (Release)
	        } else if(manual_servo_command == 2) {
	            spin_servo(&ServoR, SERVO_FORWARD_FAST);  // Right (Pull)
                spin_servo(&ServoL, SERVO_REVERSE_FAST); // Left (Release)
	        } else {
	            stop_servo(&ServoL);
	            stop_servo(&ServoR);
	        }
    }
    else {
        // Attempt at automatic gps-based control
        if (local_lastGpsUpdateTick == 0) {
            local_lastGpsUpdateTick = current_tick; // Initialize if it's zero
        }

        bool gps_signal_lost = (current_tick - local_lastGpsUpdateTick) > 2000;
        bool has_valid_fix = (myGpsData.Fix_Type > 0);

        // This logic is now handled by update_servo_runtime()
        // We MUST call this on every loop to keep the time tracking correct.
        update_servo_runtime(&ServoL);
        update_servo_runtime(&ServoR);


        osMutexAcquire(printfMutexHandle, osWaitForever);
        printf("ServoL run: %ld, ServoR run: %ld\r\n", ServoL.current_run_time, ServoR.current_run_time);
        osMutexRelease(printfMutexHandle);

        // Need to read targetLocation safely
        float local_target_lat = 0;
        if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
            local_target_lat = targetLocation.latitude;
            osMutexRelease(navDataMutexHandle);
        }

        if(has_valid_fix && local_target_lat != 0 && !gps_signal_lost)
        {
            // Add this print to prove we are in the block
            osMutexAcquire(printfMutexHandle, osWaitForever);
            printf("DEBUG: updateServos PASSED safety check.\r\n");
            osMutexRelease(printfMutexHandle);

            // Safely read shared variables using mutex
            if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
                local_target_angle = navData.target_angle;
                local_heading_real = navData.heading_real;
                osMutexRelease(navDataMutexHandle);
            } else {
                // Failed to get mutex, skip this control loop
                continue;
            }


            // CRITICAL LOGIC ERROR
            // The turn angle must be normalized to the range [-180, 180]
        	float turn_angle = normalize_angle(local_target_angle - local_heading_real);

            osMutexAcquire(printfMutexHandle, osWaitForever);
            printf("Target(int): %d, Current(int): %d, Turn(int): %d\r\n", (int)local_target_angle, (int)local_heading_real, (int)turn_angle);
            osMutexRelease(printfMutexHandle);


            if(fabs(turn_angle) >= 5) { // 5 degree deadband
                if(turn_angle < 0) {
                    // Need to turn left
                    osMutexAcquire(printfMutexHandle, osWaitForever);
                    printf("DEBUG: Calling turnLeft(%d)\r\n", (int)turn_angle);
                    osMutexRelease(printfMutexHandle);
                    turnLeft(turn_angle);
                } else if(turn_angle > 0){
                    // Need to turn right
                    osMutexAcquire(printfMutexHandle, osWaitForever);
                    printf("DEBUG: Calling turnRight(%d)\r\n", (int)turn_angle);
                    osMutexRelease(printfMutexHandle);
                    turnRight(turn_angle);
                }
            } else {
                // We are on target, go straight (stop servos)
                osMutexAcquire(printfMutexHandle, osWaitForever);
                printf("DEBUG: On target. Calling stop_servo()\r\n");
                osMutexRelease(printfMutexHandle);
                stop_servo(&ServoL);
                stop_servo(&ServoR);
            }
        } else {
            // No fix, no target, or lost signal = STOP
            stop_servo(&ServoL);
            stop_servo(&ServoR);
        }
    }

    // Reduced delay for a faster control loop (10 Hz)
    osDelay(100);
  }
}
//void moveServos(void *argument)
//{
//
//  TickType_t xLastWakeTime;
//  const TickType_t xFrequency = 1700;
//
//  // Initialise the xLastWakeTime variable with the current time.
//  xLastWakeTime = xTaskGetTickCount();
//
//  osMutexAcquire(printfMutexHandle, osWaitForever);
//  printf("MoveServos Task Started.\r\n");
//  osMutexRelease(printfMutexHandle);
//
//
//  for(;;)
//  {
//	  turnLeft(90);
//  }
//}

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

        // Protect shared variable write
        if (osMutexAcquire(navDataMutexHandle, 100) == osOK) {
            targetLocation.latitude = lat_float;
            targetLocation.longitude = lon_float;
            osMutexRelease(navDataMutexHandle);
        }

        osMutexAcquire(printfMutexHandle, osWaitForever);
        printf("New coordinate received from Xbee!\r\n");
        // Removed %f, casting to int to prevent stack corruption
        printf("  Target Lat_int:  %ld\r\n", (int32_t)(lat_float * 10000));
        printf("  Target Lon_int: %ld\r\n", (int32_t)(lon_float * 10000));
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
  uint32_t last_stats_time = 0; // This is unused, safe to ignore warning.

  // Local copies
  float local_heading_offset;
  float new_imu_heading;
  float new_heading_real;

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

            // Protect global variables that are used in sendData task
            taskENTER_CRITICAL();
            global_quat_i = get_Quat_I(&sensor1);
            global_quat_j = get_Quat_J(&sensor1);
            global_quat_k = get_Quat_K(&sensor1);
            global_quat_real = get_Quat_Real(&sensor1);

            global_accel_x = get_LinearAcceleration_X(&sensor1);
            global_accel_y = get_LinearAcceleration_Y(&sensor1);
            global_accel_z = get_LinearAcceleration_Z(&sensor1);

            global_gyro_x = get_Gyroscope_X(&sensor1);
            global_gyro_y = get_Gyroscope_Y(&sensor1);
            global_gyro_z = get_Gyroscope_Z(&sensor1);
            taskEXIT_CRITICAL();

            uint8_t accuracy = get_Quat_Accuracy(&sensor1);

            float yaw_rad = atan2f(2.0f * (global_quat_real * global_quat_k + global_quat_i * global_quat_j),
                                   1.0f - 2.0f * (global_quat_j * global_quat_j + global_quat_k * global_quat_k));
            new_imu_heading = yaw_rad * 180.0f / 3.14159265f;  // Convert to degrees
            if (new_imu_heading < 0) new_imu_heading += 360.0f;  // Normalize to 0-360
            new_imu_heading = 360.0f - new_imu_heading;

            //  Safely read the shared offset using mutex
            if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
                local_heading_offset = navData.heading_offset;
                osMutexRelease(navDataMutexHandle);
            } else {
                // use stale offset
            }


            new_heading_real = fmod(new_imu_heading + local_heading_offset, 360.0);
            if (new_heading_real < 0) new_heading_real += 360.0; // Ensure 0-360 range

            // Safely write to shared variables using mutex
            if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
                navData.imu_heading = new_imu_heading;
                navData.heading_real = new_heading_real;
                osMutexRelease(navDataMutexHandle);
            }

            uint32_t current_time = osKernelGetTickCount();
            if (current_time - last_print_time >= 500) {
                if(osMutexAcquire(printfMutexHandle, 10) == osOK) {
                    /*
                    printf("Hreal[%.3f] H[%.3f] Off[%.3f] Q[%.3f,%.3f,%.3f,%.3f] A[%.2f,%.2f,%.2f] G[%.3f,%.3f,%.3f] %s\r\n",
                           (double)new_heading_real, (double)new_imu_heading, (double)local_heading_offset, (double)global_quat_i, (double)global_quat_j, (double)global_quat_k, (double)global_quat_real,
                           (double)global_accel_x, (double)global_accel_y, (double)global_accel_z,
                           (double)global_gyro_x, (double)global_gyro_y, (double)global_gyro_z,
                           get_accuracy_string(accuracy));
                    */
                    // Print a safer, minimal log
                    printf("Hreal_int: %d, H_int: %d, Off_int: %d, Acc: %s\r\n",
                           (int)new_heading_real, (int)new_imu_heading, (int)local_heading_offset,
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


void updateHeadingOffset(void *argument)
{
    // Reduced frequency to 1s (1000ms) for faster offset updates
    // 3 seconds was far too long and would introduce massive lag
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000; // Was 3000

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	osMutexAcquire(printfMutexHandle, osWaitForever);
	printf("UpdateHeadingOffset Task Started.\r\n");
	osMutexRelease(printfMutexHandle);

    float local_imu_heading;
    float local_gps_headMot;
    float new_heading_offset;

  for(;;) {
		osMutexAcquire(printfMutexHandle, osWaitForever);
		printf("UpdateHeadingOffset Task ran\r\n");
		osMutexRelease(printfMutexHandle);

        // Safely read shared GPS data
        taskENTER_CRITICAL();
        local_gps_headMot = myGpsData.headMot;
        taskEXIT_CRITICAL();

      if(local_gps_headMot != -1) { // -1 is my init value

        //  Safely read shared IMU data using mutex
        if (osMutexAcquire(navDataMutexHandle, 100) == osOK) {
            local_imu_heading = navData.imu_heading;
            osMutexRelease(navDataMutexHandle);
        } else {
            // Failed to get mutex, skip this update
            vTaskDelayUntil( &xLastWakeTime, xFrequency);
            continue;
        }

        new_heading_offset = local_gps_headMot - local_imu_heading;
        // Normalize the offset
        new_heading_offset = normalize_angle(new_heading_offset);

        // Safely write the new offset using mutex
    	if (osMutexAcquire(navDataMutexHandle, 100) == osOK) {
            navData.heading_offset = new_heading_offset;
            osMutexRelease(navDataMutexHandle);
        }

    	osMutexAcquire(printfMutexHandle, osWaitForever);
    	printf("New heading offset (int): %d\r\n", (int)new_heading_offset);
    	osMutexRelease(printfMutexHandle);
      }

      vTaskDelayUntil( &xLastWakeTime, xFrequency);
  }
}





/*
 * @brief gather data from global variables, send over to Xbee as binary packet
 * [float] GPS Latitude
 * [float] GPS Longitude
 * [float] GPS ALtitude
 * [float] Accel X
 * [float] Accel Y
 * [float] Accel Z
 * [float] Roll Gyro
 * [float] Pitch Gyro
 * [float] Yaw Gyro
 * [float] Quat I
 * [float] Quat J
 * [float] Quat K
 * [char] '\n'
 *
 *
 */

void sendData(void *argument)
{

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 500;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	uint16_t data_size = sizeof(float) * 12;
	uint8_t buf[data_size + 1];
	float payload[12];

	osMutexAcquire(printfMutexHandle, osWaitForever);
	printf("SendData Task Started.\r\n");
	osMutexRelease(printfMutexHandle);

  for(;;) {
//	  uint16_t data_size = sizeof(float)+sizeof(float) + 1;
//	  uint8_t buf[data_size];

      taskENTER_CRITICAL();
	  payload[0] = myGpsData.Latitude;
	  payload[1] = myGpsData.Longitude;
	  payload[2] = myGpsData.Altitude;
	  payload[3] = global_accel_x;
	  payload[4] = global_accel_y;
	  payload[5] = global_accel_z;
	  payload[6] = global_gyro_x;
	  payload[7] = global_gyro_y;
	  payload[8] = global_gyro_z;
	  payload[9] = global_quat_i;
	  payload[10] = global_quat_j;
	  payload[11] = global_quat_k;
      taskEXIT_CRITICAL();

	  memcpy(buf, payload, data_size);
	  buf[data_size] = '\n';


	  xbee_send_data(buf, data_size + 1);

      if (osMutexAcquire(printfMutexHandle, 10) == osOK) {
          printf("Sent packet: Sats=%d\r\n", myGpsData.Satellites);
          osMutexRelease(printfMutexHandle);
      }


      vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

