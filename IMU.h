// 9-DOF Orientation IMU interface
/*
IMU Purpose:
    Provide orientation and motion sensing (accel, gyro, mag, fusion)

IMU Requirements:
    I2C communication for register read/write
*/

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Function pointer type for writing to an I2C register.
 */
typedef bool (*imu_write)(uint8_t addr, uint8_t reg,
                          const uint8_t *data, uint16_t len);

/**
 * @brief Function pointer type for reading from an I2C register.
 */
typedef bool (*imu_read)(uint8_t addr, uint8_t reg,
                         uint8_t *data, uint16_t len);


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
 * @brief IMU driver context for one device.
 */
typedef struct {
    imu_write write;     // I2C write function
    imu_read  read;      // I2C read function
    uint8_t   i2c_addr;  // IMU I2C address
} imu_t;

/**
 * @brief Initialize the IMU.
 *
 * @param ctx      IMU struct
 * @param addr     I2C address of the device
 * @param write_fn I2C write function
 * @param read_fn  I2C read function
 * @return true if initialization succeeded
 */
bool imu_init(imu_t *ctx, uint8_t addr,
              imu_write write_fn,
              imu_read read_fn);

/**
 * @brief Configure the IMU operating mode.
 *
 * @param ctx  IMU struct
 * @param mode Operating mode
 * @return true if config succeeded
 */
bool imu_config_mode(imu_t *ctx, uint8_t mode);

/**
 * @brief Read Euler angles (roll, pitch, yaw) in degrees.
 *
 * @param ctx   IMU struct
 * @param roll  out: roll angle
 * @param pitch out: pitch angle
 * @param yaw   out: yaw angle
 * @return true if read succeeded
 */
bool imu_read_euler(imu_t *ctx, float *roll, float *pitch, float *yaw);

/**
 * @brief Read raw accelerometer data (m/s^2).
 */
bool imu_read_accel(imu_t *ctx, float *ax, float *ay, float *az);

/**
 * @brief Read raw gyroscope data (deg/s).
 */
bool imu_read_gyro(imu_t *ctx, float *gx, float *gy, float *gz);

/**
 * @brief Read raw magnetometer data (uT).
 */
bool imu_read_mag(imu_t *ctx, float *mx, float *my, float *mz);

// ALTIMETER

/**
 * @brief Altimeter driver context for one device.
 */
typedef struct {
    i2c_write_reg write;     // I2C write function
    i2c_read_reg  read;      // I2C read function
    uint8_t       i2c_addr;  // Altimeter I2C addr
    float         p0_hPa;    // Sea-level pressure used for altitude calc
} alt_t;

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