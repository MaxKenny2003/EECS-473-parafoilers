/**
 * @file gps_header.h
 * @brief Header file for GPS data structures and functions.
 * @version 1.0
 * @date 2025-09-25
 *
 * This header defines the software interface for the u-blox NEO-M9N GPS receiver. It
 * abstracts the low-level UART communication and the parsing of NMEA or UBX binary protocol
 * messages. The application layer can initialize the module and retrieve a structured
 * navigation solution
 */

#ifndef GPS_UBLOX_NEO_M9N_H
#define GPS_UBLOX_NEO_M9N_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Represents the status of the GNSS position fix.
 */

typedef enum {
    GPS_FIX_NO_FIX = 0,       /**< No position fix */
    GPS_FIX_2D = 1,           /**< 2D position fix */
    GPS_FIX_3D = 2            /**< 3D position fix */
    GPS_DEAD_RECKONING = 3 /**< Dead reckoning fix */
} gps_fix_status_t;

/**
 * @brief Structure to hold a complete GNSS navigation solution.
 * All values are valid only if fix_type is not GPS_FIX_NO_FIX.
*/

typedef struct {
    // --- Time ---
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    // --- Position ---
    double latitude_deg;  // Latitude in decimal degrees
    double longitude_deg; // Longitude in decimal degrees
    float altitude_msl_m; // Altitude above mean sea level in meters

    // --- Velocity ---
    float speed_gnd_mps;   // Ground speed in meters per second
    float heading_gnd_deg; // Ground course heading in degrees

    // --- Quality ---
    gps_fix_type_t fix_type;    // Type of fix
    uint8_t satellites_in_view; // Number of satellites used in solution
    float pdop;                 // Position Dilution of Precision

    // --- Status ---
    bool data_is_new; // Flag indicating if data has been updated since last read

} gps_nav_solution_t;
 /**
  * @brief Initialize GPS module
  *
  * Configures the UART interface for communication with the NEO-M9N and sends any necessary
  * initial configuration commands to the module
  * @return true if initialization is successful, false otherwise
  */
bool gps_init(void);

 /**
  * @brief Processes incoming data from the GPS module
  *
  * This function should be called periodically from the main loop or a dedicated task
  * It reads bytes from the UART, parses the GNSS messages, and updates the internal
  * navigation data structure
  */
void gps_process(void);

/**
 * @brief Retrieves the latest navigation data.
 *
 * Copies the most recent, complete navigation solution into the provided data structure.
 * The data_is_new flag within the structure will be cleared after this call.
 *
 * @param[out] nav_data Pointer to a gps_nav_data_t struct to be filled with data.
 */
void gps_get_nav_data(gps_nav_data_t *nav_data);

/**
 * @brief Checks if a valid position fix is available.
 *
 * @return true if the current fix type is 2D or 3D, false otherwise.
 */
bool gps_has_fix(void);

#endif // GPS_UBLOX_NEO_M9N_H

