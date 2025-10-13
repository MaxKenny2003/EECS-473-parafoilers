/*
 * @file Hardware_Init.h
 * @brief HAL initialization functions for hardware (GPIO, Timer, SPI).
 */

#ifndef SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_HARDWARE_INIT_H_
#define SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_HARDWARE_INIT_H_

// --- Includes -------------------------------------------------------
// Includes of standard libraries
#include <stdint.h>
#include <stdio.h>

// Private includes
#include "BNO085_SPI_Error_Flags.h"
#include "Sensor_Struct.h"
#include "stm32_hal.h"
#include "stm32_hal_spi.h"

// --- Macros and data ------------------------------------------------

// *** MODIFICATION ***
// Declare a pointer to the SPI handle as 'extern'.
// This makes the pointer visible to all files that include this header.
// The pointer itself will be defined in Hardware_Init.c.
extern SPI_HandleTypeDef *hspi_ptr;

typedef struct bno085_library_spi_config_struct {
  GPIO_TypeDef *SPI_Port;
  uint16_t SPI_MISO_Pin;
  uint16_t SPI_MOSI_Pin;
  uint16_t SPI_SCK_Pin;
  SPI_TypeDef *SPI_Instance;
  uint32_t SPI_AF_mapping;
  uint32_t SPI_prescaler;
} bno085_library_spi_config_struct;

// --- Public functions -----------------------------------------------
// function prototypes
void init_GPIO_IMU(sensor_meta *sensor);

uint8_t init_HardwareBNO085(
    SPI_HandleTypeDef *hspi,
    bno085_library_spi_config_struct bno085_library_spi_config);

#endif  // SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_HARDWARE_INIT_H_
