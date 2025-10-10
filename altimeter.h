// ALTIMETER
/*
ALTIMETER Purpose:
    Acquire temperature and pressure data and convert it into altitude 

ALTIMETER Requirements:
    I2C communication for register read/write
*/

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Function pointer type for writing to an I2C register.
 */
typedef bool (*i2c_write_reg)(uint8_t addr, uint8_t reg,
    const uint8_t *data, uint16_t len);

/**
* @brief Function pointer type for reading from an I2C register.
*/
typedef bool (*i2c_read_reg)(uint8_t addr, uint8_t reg,
   uint8_t *data, uint16_t len);

/**
 * @brief Altimeter driver context for one device.
 */

// Define Function Pointers for communication protocol for Altimeter
typedef struct {
    i2c_write_reg write;     // I2C write function
    i2c_read_reg  read;      // I2C read function
    uint8_t       i2c_addr;  // Altimeter I2C addr
    float         p0_hPa;    // Sea-level pressure used for altitude calc
} alt_t;

/** 
 * @brief Altimeter Enumerator for representing the different operating modes
*/

typedef enum{
    ALT_DEEP_STANDBY = 0,
    ALT_STANDBY = 1,
    ALT_NORMAL = 2,
    ALT_CONTINUOUS = 3,
    ALT_FORCED = 4
} alt_power_mode_t;


/***************************
    COINES-WRAPPER FUNCTIONS
***************************/

/*


 I2C read function map to COINES platform

BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    return coines_read_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)length);
}


*/






/**
 * @brief Initialize the altimeter.
 *
 * @param ctx      Altimeter struct
 * @param addr     I2C address
 * @param write_fn I2C write function
 * @param read_fn  I2C read function
 * @return true if initialization/probe succeeded
 */
bool alt_init(alt_t *ctx, uint8_t addr,
              i2c_write_reg write_fn,
              i2c_read_reg  read_fn);

/**
 * @brief Set sea-level pressure reference used for altitude computation.
 *
 * @param ctx    Altimeter struct
 * @param p0_hPa Sea-level pressure (QNH) in hPa (e.g., 1013.25)
 * @return true if stored
 */
bool alt_set_sea_level(alt_t *ctx, float p0_hPa);

/**
 * @brief Read barometric pressure in Pascals (Pa).
 *
 * @param ctx Altimeter struct
 * @param pa  out: pressure in Pa
 * @return true if read succeeded
 */
bool alt_read_pressure(alt_t *ctx, float *pa);

/**
 * @brief Read temperature in degrees Celsius.
 *
 * @param ctx Altimeter struct
 * @param tc  out: temperature in °C
 * @return true if read succeeded
 */
bool alt_read_temperature(alt_t *ctx, float *tc);

/**
 * @brief Compute/read altitude in meters using current pressure and p0_hPa.
 *
 * Uses the barometric formula with the stored sea-level pressure reference.
 *
 * @param ctx Altimeter struct
 * @param m   out: altitude in meters
 * @return true if read/compute succeeded
 */
bool alt_read_altitude(alt_t *ctx, float *m);