/*
 * @file BNO085_SPI_Library.h
 */

#ifndef SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_BNO085_SPI_LIBRARY_H_
#define SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_BNO085_SPI_LIBRARY_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Private includes - CORRECTED ORDER
#include "Sensor_Struct.h" // Defines sensor_meta first
#include "BNO085_SPI_Error_Flags.h"
#include "BNO085_SPI_Shims.h"
#include "Hardware_Init.h"

/*
 * ====================================================================
 * NEW DEFINITIONS FOR TARE COMMAND
 * ====================================================================
 * These macros provide clear names for the parameters used in the
 * tare_now function, making the code more readable and preventing
 * "magic numbers".
 */
// Defines for tare_now function axes_to_tare parameter
#define TARE_AXES_Z (0x04)
#define TARE_AXES_ALL (0x07)

// Defines for tare_now function rotation_vector_basis parameter
#define TARE_BASIS_ROT_VEC (0x00)
#define TARE_BASIS_GAME_ROT_VEC (0x01)
#define TARE_BASIS_GEOMAG_ROT_VEC (0x02)
#define TARE_BASIS_GYRO_INT_ROT_VEC (0x03)
#define TARE_BASIS_ARVR_STAB_ROT_VEC (0x04)
#define TARE_BASIS_ARVR_STAB_GAME_ROT_VEC (0x05)


// --- Public functions -----------------------------------------------
void register_Sensor(sensor_meta *sensor, uint8_t sensor_number,
                     uint16_t sensor_CSN_Pin, GPIO_TypeDef *sensor_CSN_Port,
                     uint16_t sensor_INTN_Pin, GPIO_TypeDef *sensor_INTN_Port,
                     uint16_t sensor_RSTN_Pin, GPIO_TypeDef *sensor_RSTN_Port);
uint8_t clear_init_Message_IMU(sensor_meta *sensor);
void hardreset_IMU(sensor_meta *sensor);
bool get_and_clear_Reset_Status(sensor_meta *sensor);
uint8_t enable_GameRotationVector(sensor_meta *sensor,
                                  uint16_t time_between_reports);
uint8_t enable_LinearAcceleration(sensor_meta *sensor,
                                  uint16_t time_between_reports);
uint8_t enable_Gyroscope(sensor_meta *sensor,
                         uint16_t time_between_reports);

/*
 * ====================================================================
 * NEW FUNCTION PROTOTYPE FOR TARE COMMAND
 * ====================================================================
 * This declares the tare_now function so that other files, like
 * freertos.c, can see and use it without compiler errors.
 */
uint8_t tare_now(sensor_meta *sensor, uint8_t axes_to_tare, uint8_t rotation_vector_basis);

bool data_available(sensor_meta *sensor);
float get_LinearAcceleration_X(sensor_meta *sensor);
float get_LinearAcceleration_Y(sensor_meta *sensor);
float get_LinearAcceleration_Z(sensor_meta *sensor);
float get_Gyroscope_X(sensor_meta *sensor);
float get_Gyroscope_Y(sensor_meta *sensor);
float get_Gyroscope_Z(sensor_meta *sensor);
uint8_t get_Gyroscope_Accuracy(sensor_meta *sensor);
float get_Quat_I(sensor_meta *sensor);
float get_Quat_J(sensor_meta *sensor);
float get_Quat_K(sensor_meta *sensor);
float get_Quat_Real(sensor_meta *sensor);
uint8_t get_Quat_Accuracy(sensor_meta *sensor);
uint8_t check_Connection_IMU(sensor_meta *sensor);
uint8_t check_Command_Success(sensor_meta *sensor, uint8_t status_command);


#endif /* SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_BNO085_SPI_LIBRARY_H_ */
