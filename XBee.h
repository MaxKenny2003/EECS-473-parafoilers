//XBee interface

/*
XBee Purpose:
    Send and receive data between payload and groundstation

XBee Requiremnts:
    Quick data transmission and medium-long range
*/
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Set up how to write one byte to the UART connected to the XBee.
 */
typedef bool (*xbee_write)(uint8_t byte);

/**
 * @brief Set up how to read one byte from the UART connected to the XBee (non-blocking).
 */
typedef bool (*xbee_read)(uint8_t *out);

/**
 * @brief XBee struct for each device
 */
typedef struct {
    xbee_write write;    // UART write function
    xbee_read  read;     // UART read function
} xbee_t;

/**
 * @brief Initialize the XBee
 * 
 * @param ctx       XBee struct
 * @param write_fn  write function
 * @param read_fn   read function
 */
void xbee_init(xbee_t *ctx,
          xbee_write write_fn,
          xbee_read read_fn);

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
bool xbee_send_data(xbee_t *ctx,
          uint64_t dest64, 
          uint16_t dest16, 
          const uint8_t *payload, 
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
bool xbee_receive_data(xbee_t *ctx,
          uint64_t *src64, 
          uint8_t *buffer,
          uint16_t max_len,
          uint16_t *out_len);

/**
 * @brief Tells the XBee which Personal Area Network (PAN) ID to join
 * 
 * @return Returns True if config is successful
 */
bool xbee_config_PAN(xbee_t *ctx, const uint8_t pan_id[8]);

/**
 * @brief Tells the XBee which RF channel (frequency slot in the 2.4 GHz band) to use.
 * 
 * @return Returns True if config is successful
 */
bool xbee_config_channel(xbee_t *ctx, uint8_t channel);