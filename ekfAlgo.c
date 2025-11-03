/*
 * ekfAlgo.c
 *
 *  Created on: Oct 21, 2025
 *      Author: tommytt427
 */

#include "ekfAlgo.h"
#include <string.h>


//private helper macros
#define SQ(x) ((x)*(x))


// matrix indexing macro: access P[row][col]
#define P(ekf, i, j) (ekf->P[i][j])

/*
 * Initialize the Parafoil EKF
 */
void ParafoilEKF_Init(ParafoilEKF *ekf, float Pinit, float *Q, float *R) {
    /* Zero out the structure */
    memset(ekf, 0, sizeof(ParafoilEKF));
    
    /* Initialize state estimates to zero */
    ekf->pos_n = 0.0f;
    ekf->pos_e = 0.0f;
    ekf->pos_d = 0.0f;
    ekf->vel_n = 0.0f;
    ekf->vel_e = 0.0f;
    ekf->vel_d = 0.0f;
    ekf->heading_bias = 0.0f;
    
    /* Initialize covariance matrix P - diagonal only */
    for (int i = 0; i < EKF_NUM_STATES; i++) {
        for (int j = 0; j < EKF_NUM_STATES; j++) {
            if (i == j) {
                P(ekf, i, j) = Pinit;  // Diagonal elements
            } else {
                P(ekf, i, j) = 0.0f;   // Off-diagonal elements
            }
        }
    }
    
    /* Copy process noise Q */
    for (int i = 0; i < EKF_NUM_STATES; i++) {
        ekf->Q[i] = Q[i];
    }
    
    /* Copy measurement noise R */
    for (int i = 0; i < EKF_NUM_MEASUREMENTS; i++) {
        ekf->R[i] = R[i];
    }
    
    /* GPS reference not initialized yet */
    ekf->ref_lat = 0.0;
    ekf->ref_lon = 0.0;
    ekf->ref_alt = 0.0;
    ekf->initialized = 0;  // Will be set to 1 after first GPS fix
    ekf->last_gps_time = 0;
}

/*
 * Convert GPS lat/lon to NED coordinates (flat-earth approximation)
 * Accurate for distances < 100km
 */
void GPS_ToNED(double lat, double lon, float alt,
               double ref_lat, double ref_lon, float ref_alt,
               float *north, float *east, float *down) {
    
    /* Convert degrees to radians */
    double lat_rad = lat * DEG_TO_RAD;
    double lon_rad = lon * DEG_TO_RAD;
    double ref_lat_rad = ref_lat * DEG_TO_RAD;
    double ref_lon_rad = ref_lon * DEG_TO_RAD;
    
    /* Compute differences */
    double dlat = lat_rad - ref_lat_rad;
    double dlon = lon_rad - ref_lon_rad;
    
    /* Convert to meters (flat earth approximation) */
    *north = (float)(dlat * EARTH_RADIUS);
    *east = (float)(dlon * EARTH_RADIUS * cos(ref_lat_rad));
    *down = -(alt - ref_alt);  // NED: positive down
}

/*
 * Convert NED coordinates back to GPS lat/lon
 */
void NED_ToGPS(float north, float east, float down,
               double ref_lat, double ref_lon, float ref_alt,
               double *lat, double *lon, float *alt) {
    
    double ref_lat_rad = ref_lat * DEG_TO_RAD;
    double ref_lon_rad = ref_lon * DEG_TO_RAD;
    
    /* Convert meters to radians */
    double dlat = north / EARTH_RADIUS;
    double dlon = east / (EARTH_RADIUS * cos(ref_lat_rad));
    
    /* Convert back to degrees */
    *lat = (ref_lat_rad + dlat) * RAD_TO_DEG;
    *lon = (ref_lon_rad + dlon) * RAD_TO_DEG;
    *alt = ref_alt - down;  // Convert NED down back to altitude
}

/*
 * EKF Prediction Step
 * 
 * This is the heart of the filter - runs at 100 Hz
 */

