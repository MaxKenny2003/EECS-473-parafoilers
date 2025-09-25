// 9-DOF Orientation IMU interface
/*
IMU Purpose:
    Provide orientation and motion sensing (accel, gyro, mag, fusion)

IMU Requirements:
    I2C communication for register read/write
    Delay for startup/config (optional)
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
 * @brief Function pointer type for delay in milliseconds (optional).
 */
typedef void (*imu_delay)(uint32_t ms);

/**
 * @brief IMU driver context for one device.
 */
typedef struct {
    imu_write write;     // I2C write function
    imu_read  read;      // I2C read function
    imu_delay delay;     // Optional delay function
    uint8_t   i2c_addr;  // IMU I2C address
} imu_t;

/**
 * @brief Initialize the IMU.
 *
 * @param ctx      IMU struct
 * @param addr     I2C address of the device
 * @param write_fn I2C write function
 * @param read_fn  I2C read function
 * @param delay_fn Optional delay function (may be NULL)
 * @return true if initialization succeeded
 */
bool imu_init(imu_t *ctx, uint8_t addr,
              imu_write write_fn,
              imu_read read_fn,
              imu_delay delay_fn);

/**
 * @brief Configure the IMU operating mode.
 *
 * @param ctx  IMU struct
 * @param mode Operating mode (e.g., config, NDOF, IMU-only)
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
