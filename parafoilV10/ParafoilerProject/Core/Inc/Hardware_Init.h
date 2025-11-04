/* CORRECTED FILE: Hardware_Init.h */

#ifndef SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_HARDWARE_INIT_H_
#define SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_HARDWARE_INIT_H_

#include <stdint.h>
#include <stdio.h>

// CORRECTED Private includes - Order is critical
#include "main.h" // Provides all HAL types
#include "BNO085_SPI_Error_Flags.h"

// Forward declare the struct to break the circular dependency
struct sensor_meta;

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

// Use the forward-declared struct name in the function prototype
void init_GPIO_IMU(struct sensor_meta *sensor);

uint8_t init_Hardware_BNO085(
    SPI_HandleTypeDef *hspi,
    bno085_library_spi_config_struct bno085_library_spi_config);

#endif
