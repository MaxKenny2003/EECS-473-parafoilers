#ifndef __NEO_M9N_H
#define __NEO_M9N_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define RX_BUFFER_LEN 512

// --- UBX NAV-PVT Message Structure ---
// This structure must be packed to ensure there is no padding between members,
// which would misalign it with the raw byte buffer from the GPS.
#pragma pack(push, 1)
typedef struct {
    uint32_t iTOW;      // GPS time of week of the navigation epoch (ms)
    uint16_t year;      // Year (UTC)
    uint8_t  month;     // Month, range 1..12 (UTC)
    uint8_t  day;       // Day of month, range 1..31 (UTC)
    uint8_t  hour;      // Hour of day, range 0..23 (UTC)
    uint8_t  min;       // Minute of hour, range 0..59 (UTC)
    uint8_t  sec;       // Second of minute, range 0..60 (UTC)
    uint8_t  valid;     // Validity Flags
    uint32_t tAcc;      // Time accuracy estimate (ns)
    int32_t  nano;      // Fraction of second, range -1e9..1e9 (ns)
    uint8_t  fixType;   // GNSSfix Type: 0..5
    uint8_t  flags;     // Fix Status Flags
    uint8_t  flags2;    // Additional Flags
    uint8_t  numSV;     // Number of satellites used in Nav Solution
    int32_t  lon;       // Longitude (deg * 1e-7)
    int32_t  lat;       // Latitude (deg * 1e-7)
    int32_t  height;    // Height above Ellipsoid (mm)
    int32_t  hMSL;      // Height above mean sea level (mm)
    uint32_t hAcc;      // Horizontal Accuracy Estimate (mm)
    uint32_t vAcc;      // Vertical Accuracy Estimate (mm)
    int32_t  velN;      // NED north velocity (mm/s)
    int32_t  velE;      // NED east velocity (mm/s)
    int32_t  velD;      // NED down velocity (mm/s)
    int32_t  gSpeed;    // Ground Speed (2-D) (mm/s)
    int32_t  headMot;   // Heading of motion (2-D) (deg * 1e-5)
    uint32_t sAcc;      // Speed Accuracy Estimate (mm/s)
    uint32_t headAcc;   // Heading Accuracy Estimate (deg * 1e-5)
    uint16_t pDOP;      // Position DOP
    uint8_t  reserved1[6];
    int32_t  headVeh;   // Heading of vehicle (2-D) (deg * 1e-5)
    uint8_t  reserved2[4];
} UBX_NAV_PVT_t;
#pragma pack(pop)


// This is the main data structure for our application
typedef	struct GPS_DATA {
	uint8_t newData;
	float Latitude;
	float Longitude;
	float Altitude;
	float  headMot;
	float headAcc;
	uint8_t Satellites;
	uint8_t Fix_Type;
} GPS_DATA_t;

// A simple struct for holding coordinates.
typedef struct {
    float latitude;
    float longitude;
} GPS_Info;


extern volatile uint8_t gpsRxBuffer[RX_BUFFER_LEN];
extern GPS_DATA_t myGpsData;
extern volatile uint16_t gps_data_length;

// This function parses the data in the provided buffer
void processGPS(const uint8_t* p_data, uint16_t length);

// This function sends the configuration message to the GPS and starts DMA reception.
void M9N_Init(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __NEO_M9N_H */

