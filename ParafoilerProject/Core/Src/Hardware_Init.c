/*
 * @file Hardware_Init.c
 * @brief HAL initialization functions for hardware (GPIO, Timer, SPI).
 */

// --- Includes -------------------------------------------------------
// Private includes
#include "Hardware_Init.h"
#include "main.h"
// --- Macros and data ------------------------------------------------

// *** MODIFICATION ***
// Define the global SPI handle pointer. This is the single definition
// that the linker will use for the `extern` declaration in the header.
SPI_HandleTypeDef *hspi_ptr;

// --- Private methods ------------------------------------------------

/**
 * @brief Init GPIOs for sensor
 * @note This function is now simplified as most initialization is done by CubeIDE.
 * It primarily ensures the RST pin is set up correctly before a hard reset.
 */
static void HAL_Init_GPIO_Sensor(ports_pins ports_pins_config) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // The other pins (CS, INT) are configured by MX_GPIO_Init().
  // We only need to ensure RSTN is configured as an output here for the hardreset function.

  /* Configure RSTN*/
  HAL_GPIO_WritePin(ports_pins_config.RSTN_Port, ports_pins_config.RSTN_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = ports_pins_config.RSTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ports_pins_config.RSTN_Port, &GPIO_InitStruct);
}


// --- Public functions -------------------------------------------------

/**
 * @brief Initializes GPIOs for IMUs
 * @param *sensor: Pointer to corresponding sensor meta data
 */
void init_GPIO_IMU(sensor_meta *sensor) {
  HAL_Init_GPIO_Sensor(sensor->ports_pins);
}

/**
 * @brief Links the library's internal SPI pointer to the one initialized in main.c
 */
uint8_t init_Hardware_BNO085(
    SPI_HandleTypeDef *hspi_handle_from_main,
    bno085_library_spi_config_struct bno085_library_spi_config) {
  // *** MODIFICATION ***
  // Assign the address of the hspi1 handle from main.c to our global pointer.
  // Now, any library file that includes Hardware_Init.h can use hspi_ptr
  // to perform SPI operations.
  hspi_ptr = hspi_handle_from_main;

  // The bno085_library_spi_config is not used in this simplified setup
  // as CubeIDE handles the SPI peripheral initialization.
  (void)bno085_library_spi_config;

  return N_ERR;
}
