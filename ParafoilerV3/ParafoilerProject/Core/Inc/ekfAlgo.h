/*
 * ekfAlgo.h
 *
 *  Created on: Oct 21, 2025
 *      Author: tonym
 */

#ifndef INC_EKFALGO_H_
#define INC_EKFALGO_H_



/* Includes ------------------------------------------------------------------*/
//#include "stm32h7xx_hal.h"
//#include "BNO085_SPI_Library.h"
//#include "Hardware_Init.h"
#include <stdint.h>
#include <math.h>

/* Constants */
#define GRAVITY 9.81f              // m/s^2
#define EARTH_RADIUS 6378137.0f    // meters
#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define M_PI       3.14159265358979323846   // pi
/* State dimensions */
#define EKF_NUM_STATES 7           // pos(3) + vel(3) + heading_bias(1)
#define EKF_NUM_MEASUREMENTS 7     // GPS: pos(3) + vel(2) + heading(1) + vd(1)

/**
 * 7 measurements because we need these data:
 * GPS: North, east, down, north velocity, east velocity, down velocity (which is often just 0), heading (for bias correction)
 */



/* Variable Definitions */
typedef struct {
    /* State estimates - NED (North-East-Down) frame */
    float pos_n;           // North position (m)
    float pos_e;           // East position (m)
    float pos_d;           // Down position (m)
    float vel_n;           // North velocity (m/s)
    float vel_e;           // East velocity (m/s)
    float vel_d;           // Down velocity (m/s)
    float heading_bias;    // Heading bias correction (rad)
    
    /* Covariance matrix P (7x7) - State uncertainty */
    float P[EKF_NUM_STATES][EKF_NUM_STATES];
    
    /* Process noise covariance Q (diagonal only for efficiency) */
    float Q[EKF_NUM_STATES];
    
    /* Measurement noise covariance R (diagonal only) */
    float R[EKF_NUM_MEASUREMENTS];
    
    /* GPS reference point for NED conversion */
    double ref_lat;        // Reference latitude (deg)
    double ref_lon;        // Reference longitude (deg)
    float ref_alt;         // Reference altitude (m)
    
    /* Filter status */
    uint8_t initialized;   // 1 = filter ready, 0 = needs initialization
    uint32_t last_gps_time; // Timestamp of last GPS update (ms)
} ParafoilEKF;

/*
 * Initialize the Parafoil EKF
 * 
 * Parameters:
 *   ekf         - Pointer to EKF structure
 *   Pinit       - Initial state covariance (scalar, applied to all states)
 *   Q           - Process noise array [7] (pos, pos, pos, vel, vel, vel, bias)
 *   R           - Measurement noise array [7] (gps_pos, gps_pos, gps_pos, 
 *                                              gps_vel, gps_vel, gps_vel, heading)
 * 
 * Note: Call this ONCE at startup before any prediction/update
 */
void ParafoilEKF_Init(ParafoilEKF *ekf, float Pinit, float *Q, float *R);

/*
 * EKF Prediction Step - Run at 100 Hz (every 10ms)
 * 
 * Uses BNO085 quaternion and linear acceleration to predict state changes
 * 
 * Parameters:
 *   ekf         - Pointer to EKF structure
 *   quat        - Quaternion from BNO085 [qw, qx, qy, qz] (normalized)
 *   accel_body  - Linear acceleration in body frame [ax, ay, az] (m/s^2)
 *                 NOTE: This should be LINEAR accel (gravity already removed by BNO085)
 *   dt          - Time step since last prediction (seconds)
 * 
 * Process:
 *   1. Rotate body acceleration to NED frame using quaternion
 *   2. Integrate acceleration to update velocity
 *   3. Integrate velocity to update position
 *   4. Propagate covariance uncertainty
 */
void ParafoilEKF_Predict(ParafoilEKF *ekf, 
                         float *quat,       // [qw, qx, qy, qz]
                         float *accel_body, // [ax, ay, az] body frame
                         float dt);         // seconds


/*
 * EKF Update Step - Run when GPS data available (~1-5 Hz)
 * 
 * Uses GPS position, velocity, and heading to correct state estimates
 * 
 * Parameters:
 *   ekf            - Pointer to EKF structure
 *   gps_lat        - GPS latitude (degrees)
 *   gps_lon        - GPS longitude (degrees)
 *   gps_alt        - GPS altitude (meters above MSL)
 *   gps_speed      - GPS ground speed (m/s)
 *   gps_heading    - GPS course over ground (degrees, 0-360)
 *   quat           - Current quaternion from BNO085 [qw, qx, qy, qz]
 *   valid_heading  - 1 if GPS heading is valid (speed > 2 m/s), 0 otherwise
 * 
 * Process:
 *   1. Convert GPS lat/lon to NED coordinates
 *   2. Compute GPS velocity vector from speed + heading
 *   3. Compute heading bias error
 *   4. Apply Kalman update to correct state
 * 
 * Returns:
 *   0  = Update successful
 *   -1 = Update failed (invalid data or not initialized)
 */
int8_t ParafoilEKF_UpdateGPS(ParafoilEKF *ekf,
                             double gps_lat,
                             double gps_lon,
                             float gps_alt,
                             float gps_speed,
                             float gps_heading,
                             float *quat,
                             uint8_t valid_heading);




//Helper functions below

/**
 * Get current heading estimate (corrected for bias)
 * Parameters:
 *  ekf - Pointer to EKF structure
 * quat - Current quaternion from BNO085 [qw, qx, qy, qz]
 * Returns:
 * Heading in degrees (0-360)
 */
float ParafoilEKF_GetHeading(ParafoilEKF *ekf, float *quat);

/*
 * Get position in GPS coordinates
 * 
 * Converts NED position back to lat/lon/alt
 * 
 * Parameters:
 *   ekf      - Pointer to EKF structure
 *   gps_lat  - Output: Latitude (degrees)
 *   gps_lon  - Output: Longitude (degrees)
 *   gps_alt  - Output: Altitude (meters)
 */
void ParafoilEKF_GetPositionGPS(ParafoilEKF *ekf,
                                double *gps_lat,
                                double *gps_lon,
                                float *gps_alt);


/*
 * Get velocity in NED frame
 * 
 * Parameters:
 *   ekf    - Pointer to EKF structure
 *   vel_n  - Output: North velocity (m/s)
 *   vel_e  - Output: East velocity (m/s)
 *   vel_d  - Output: Down velocity (m/s)
 */
void ParafoilEKF_GetVelocity(ParafoilEKF *ekf,
                             float *vel_n,
                             float *vel_e,
                             float *vel_d);


/*
 * Check if EKF is initialized and has valid GPS reference
 * 
 * Returns:
 *   1 = Ready to use
 *   0 = Not initialized or no GPS reference
 */
uint8_t ParafoilEKF_IsReady(ParafoilEKF *ekf);


// utility functions for coordinate conversions
/*
 * Utility: Convert GPS lat/lon to NED position
 * (Internal helper, can be called externally if needed)
 */
void GPS_ToNED(double lat, double lon, float alt,
               double ref_lat, double ref_lon, float ref_alt,
               float *north, float *east, float *down);


/* Utility: Convert NED position to GPS lat/lon
 * (Internal helper, can be called externally if needed)
 */
void NED_ToGPS(float north, float east, float down,
               double ref_lat, double ref_lon, float ref_alt,
               double *lat, double *lon, float *alt);


/* Utility: Wrap angle to [-pi, pi]
 * (Internal helper, can be called externally if needed)
 */
static inline float wrap_to_pi(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}


#endif /* INC_EKFALGO_H_ */
