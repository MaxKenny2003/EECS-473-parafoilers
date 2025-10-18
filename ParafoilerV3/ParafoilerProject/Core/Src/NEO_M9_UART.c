#include "NEO_M9_UART.h"
#include <string.h>
#include <stdio.h>
#include "stm32h7xx_hal.h" // Required for HAL_Delay

// This buffer is the destination for the DMA transfer.
volatile uint8_t gpsRxBuffer[RX_BUFFER_LEN];

// This struct holds the parsed GPS data.
GPS_DATA_t myGpsData;

/**
  * @brief  Calculates the UBX checksum.
  * @param  p_buff: Pointer to the data buffer.
  * @param  length: Length of the data to checksum.
  * @retval None
  */
static void calculate_checksum(const uint8_t *p_buff, uint16_t length, uint8_t *p_ck_a, uint8_t *p_ck_b) {
    *p_ck_a = 0;
    *p_ck_b = 0;
    for (uint16_t i = 0; i < length; i++) {
        *p_ck_a += p_buff[i];
        *p_ck_b += *p_ck_a;
    }
}

/**
  * @brief  Configures the NEO-M9N to output UBX-NAV-PVT messages.
  * @param  huart: Pointer to the UART handle.
  * @retval None
  */
void M9N_Init(UART_HandleTypeDef *huart) {
    printf("Configuring NEO-M9N for UBX-only output...\r\n");

    // FIX HERE, because osDelay is not enabled yet by scheduler, we swap to HAL_Delay
    HAL_Delay(500);

    // ===== FIRST: Disable ALL NMEA messages =====
    // UBX-CFG-VALSET to disable all NMEA on UART1
    uint8_t disable_nmea[] = {
        0xB5, 0x62,  // Header
        0x06, 0x8A,  // CFG-VALSET
        0x44, 0x00,  // Length: 68 bytes
        0x00,        // Version
        0x01,        // RAM layer only
        0x00, 0x00,  // Reserved

        // CFG-MSGOUT-NMEA_ID_GGA_UART1 (disable)
        0xBB, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_GLL_UART1 (disable)
        0xC0, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_GSA_UART1 (disable)
        0xC5, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_GSV_UART1 (disable)
        0xCA, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_RMC_UART1 (disable)
        0xAC, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_VTG_UART1 (disable)
        0xB1, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_GRS_UART1 (disable)
        0xCF, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_GST_UART1 (disable)
        0xD4, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_ZDA_UART1 (disable)
        0xD9, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_GBS_UART1 (disable)
        0xDD, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_DTM_UART1 (disable)
        0xE2, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_GNS_UART1 (disable)
        0xB6, 0x00, 0x91, 0x20, 0x00,
        // CFG-MSGOUT-NMEA_ID_VLW_UART1 (disable)
        0xE7, 0x00, 0x91, 0x20, 0x00,

        0x00, 0x00  // Placeholder for checksum
    };

    uint16_t payload_len = sizeof(disable_nmea) - 8;
    // The payload starts after the header (2) and msg class/id (2), and ends before the checksum (2)
    // The checksum calculation includes the class, id, and length fields.
    calculate_checksum(&disable_nmea[2], payload_len + 4,
                      &disable_nmea[payload_len + 6],
                      &disable_nmea[payload_len + 7]);

    HAL_UART_Transmit(huart, disable_nmea, sizeof(disable_nmea), 1000);
    HAL_Delay(250);

    // ===== SECOND: Enable UBX-NAV-PVT =====
    uint8_t enable_pvt[] = {
        0xB5, 0x62,  // Header
        0x06, 0x8A,  // CFG-VALSET
        0x09, 0x00,  // Length: 9 bytes
        0x00,        // Version
        0x01,        // RAM layer only
        0x00, 0x00,  // Reserved

        // CFG-MSGOUT-UBX_NAV_PVT_UART1 (enable, 1 message per solution)
        0x07, 0x00, 0x91, 0x20, 0x01,

        0x00, 0x00  // Placeholder for checksum
    };

    payload_len = sizeof(enable_pvt) - 8;
    calculate_checksum(&enable_pvt[2], payload_len + 4,
                      &enable_pvt[payload_len + 6],
                      &enable_pvt[payload_len + 7]);

    HAL_UART_Transmit(huart, enable_pvt, sizeof(enable_pvt), 1000);
    HAL_Delay(250);

    printf("GPS configuration complete.\r\n");

    // Flush any old data from the hardware buffer before starting DMA
    __HAL_UART_FLUSH_DRREGISTER(huart);

    // NOTE: The DMA reception is now started in the RTOS task itself,
    // not here. This function is for configuration only.
}


/**
  * @brief  Processes the received GPS data (assumes UBX protocol).
  * @param  p_data: Pointer to the data buffer to process.
  * @param  length: Length of the data in the buffer.
  * @retval None
  */
void processGPS(const uint8_t* p_data, uint16_t length) {
    myGpsData.newData = 0;

    // Search for UBX header
    for (uint16_t i = 0; i < length - 8; i++) {
        // Look for proper UBX sync bytes
        if (p_data[i] == 0xB5 && p_data[i+1] == 0x62) {
            const uint8_t* msg_start = &p_data[i];
            uint16_t remaining = length - i;

            // Verify it's NAV-PVT (Class 0x01, ID 0x07)
            if (remaining >= 8 && msg_start[2] == 0x01 && msg_start[3] == 0x07) {
                uint16_t payload_length = (msg_start[5] << 8) | msg_start[4];

                if (payload_length != 92) {
                    continue;  // Wrong payload size, keep searching
                }

                uint16_t total_msg_len = payload_length + 8;  // header(2)+class/id(2)+len(2) + payload + checksum(2)

                if (remaining < total_msg_len) {
                    // Not a full message in this buffer, wait for the next DMA transfer
                    return;
                }

                // Verify checksum
                uint8_t ck_a, ck_b;
                calculate_checksum(&msg_start[2], payload_length + 4, &ck_a, &ck_b);

                if (ck_a == msg_start[payload_length + 6] &&
                    ck_b == msg_start[payload_length + 7]) {

                    // Valid message - parse it
                    UBX_NAV_PVT_t *pvt = (UBX_NAV_PVT_t *)&msg_start[6];

                    myGpsData.Fix_Type = pvt->fixType;
                    myGpsData.Satellites = pvt->numSV;

                    if (pvt->fixType >= 3) { // 3 = 3D fix
                        myGpsData.Latitude = pvt->lat * 1e-7f;
                        myGpsData.Longitude = pvt->lon * 1e-7f;
                        myGpsData.Altitude = pvt->hMSL * 1e-3f;
                        myGpsData.newData = 1;
                    }
                    return;  // Successfully processed one message, exit
                }
            }
        }
    }
}
