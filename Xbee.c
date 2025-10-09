//XBee interface

//PAN ID = 0x7C3A

#include "Xbee.h"
#include <stdio.h>
#include <math.h>
#include "Servo.h"


#define XBEE_RX_BUFFER_SIZE 9


UART_HandleTypeDef* uart;

uint8_t rx_buf[XBEE_RX_BUFFER_SIZE];

uint8_t MODE = 0; //Auto mode

/**
 * @brief Initialize the XBee
 *
 * @param ctx       XBee struct
 * @param write_fn  write function
 * @param read_fn   read function
 */
void xbee_init(UART_HandleTypeDef *huart) {
    //setup UART
    //setup write and red
    //setup data structures
	uart = huart;

    HAL_UART_Receive_IT(uart, rx_buf, XBEE_RX_BUFFER_SIZE);
}

//int DIN_PIN, int DOUT_PIN, int CTS_PIN, int RTS_PIN

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
bool xbee_send_data(uint8_t *payload, uint16_t length) {
    //Uart write
    HAL_UART_Transmit(uart, payload, length, 1000);

    return true;
}

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
bool xbee_receive_data(uint8_t *buffer, uint16_t max_len) {
    //Not needed maybe?
	HAL_UART_Receive(uart, buffer, max_len, HAL_MAX_DELAY);

	return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == uart->Instance) {
        handle_cmd(rx_buf);
    }

    HAL_UART_Receive_IT(uart, rx_buf, XBEE_RX_BUFFER_SIZE);
}


// Change when packet strcuture is finalized
void handle_cmd(uint8_t *cmd) {
    char cmd_type = cmd[0];

    if(cmd_type == 'C') {
        //coordinate
    	int32_t lat = (cmd[1] << 24) + (cmd[2]<<16) + (cmd[3]<<8) + cmd[4];
    	int32_t lon = (cmd[5] << 24) + (cmd[6]<<16) + (cmd[7]<<8) + cmd[8];

    	float lat_float = (float)lat / pow(10,7);
    	float lon_float = (float)lon / pow(10,7);

    	printf("Coordinate: %f %f", lat_float, lon_float);
    } else if(cmd_type == 'L' || cmd_type == 'R' ) {
        printf("Moving: %c", cmd[0]);

        if(cmd_type == 'L') {
        	spin_servo(165);
        } else {
        	spin_servo(135);
        }
    } else if(cmd_type == 'M') {
        //manual control
    } else {
        printf("Error: unrecognized cmd\r\n");
    }
}
