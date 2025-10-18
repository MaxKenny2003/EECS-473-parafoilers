//XBee interface

/*
XBee Purpose:
    Send and receive data between payload and groundstation

XBee Requiremnts:
    Quick data transmission and medium-long range
*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


#include "stm32h7xx_hal.h"




/**
 * @brief Initialize the XBee
 *
 * @param ctx       XBee struct
 * @param write_fn  write function
 * @param read_fn   read function
 */
void xbee_init(UART_HandleTypeDef *huart);

/**
 * @brief Send data from one Xbee to another.
 *
 * @param ctx       XBee struct
 * @param dest64    Destination location
 * @param dest16    Network Address
 * @param payload   Pointer to data to be sent
 * @param length    Length of data
 * @return Returns True if data is sent
 */
bool xbee_send_data(uint8_t *payload,
          uint16_t length);

/**
 * @brief Read data from an Xbee.
 *
 * @param ctx        Receivng XBee struct
 * @param src64      Sender's address
 * @param buffer     Store data from Sender
 * @param max_len    Max length of data to accept
 * @param out_len    Length of data received
 * @return Returns True if data is received
 */
bool xbee_receive_data(uint8_t *buffer,
          uint16_t max_len);

void handle_cmd(uint8_t *cmd);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
extern volatile uint8_t new_coord_flag;
uint32_t xbee_get_interrupt_count(void);
uint32_t xbee_get_restart_count(void);
extern volatile uint8_t gps_error_recovery_in_progress;
// This function is the single error callback for ALL uarts in the project.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
