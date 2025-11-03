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
void ParafoilEKF_Predict(ParafoilEKF *ekf,
                         float *quat, // [qw, qx, qy, qz]
                         float *accel_body, // [ax, ay, az] in body frame
                         float dt) {
    if (!ekf->initialized) return;

    // --- 1. Rotate body-frame accel to NED frame ---
    // NOTE: This uses the BNO085's "Linear Acceleration" which already has gravity removed
    // This is why we do not add gravity in this step
    
    // This is v_ned = R(q) * v_body
    // Where R(q) is the rotation matrix from body to NED
    float q0=quat[0], q1=quat[1], q2=quat[2], q3=quat[3];
    float ax=accel_body[0], ay=accel_body[1], az=accel_body[2];

    // Pre-calculate squares
    float q0q0 = q0*q0;
    float q1q1 = q1*q1;
    float q2q2 = q2*q2;
    float q3q3 = q3*q3;
    
    // Pre-calculate products
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q3 = q2*q3;

    float ax_ned = (q0q0 + q1q1 - q2q2 - q3q3)*ax + (2*q1q2 - 2*q0q3)*ay + (2*q1q3 + 2*q0q2)*az;
    float ay_ned = (2*q1q2 + 2*q0q3)*ax + (q0q0 - q1q1 + q2q2 - q3q3)*ay + (2*q2q3 - 2*q0q1)*az;
    float az_ned = (2*q1q3 - 2*q0q2)*ax + (2*q2q3 + 2*q0q1)*ay + (q0q0 - q1q1 - q2q2 + q3q3)*az;

    // --- 2. Update state estimates (Euler integration) ---
    // Note: This is a simple integration. A more complex one is:
    // ekf->pos_n += ekf->vel_n * dt + 0.5f * ax_ned * dt * dt;
    // But this is fine for high-frequency (100Hz) updates
    
    // Update position: p = p + v*dt
    ekf->pos_n += ekf->vel_n * dt;
    ekf->pos_e += ekf->vel_e * dt;
    ekf->pos_d += ekf->vel_d * dt;
    
    // Update velocity: v = v + a*dt
    ekf->vel_n += ax_ned * dt;
    ekf->vel_e += ay_ned * dt;
    ekf->vel_d += az_ned * dt;

    // heading_bias remains constant (it's a random walk, handled by Q)

    // --- 3. Propagate covariance matrix P ---
    // P(k+1) = P(k) + dt * (A*P + P*A' + Q)
    // Where A is the state transition Jacobian
    //      [ 0 0 0 1 0 0 0 ]
    //      [ 0 0 0 0 1 0 0 ]
    //      [ 0 0 0 0 0 1 0 ]
    // A =  [ 0 0 0 0 0 0 0 ]
    //      [ 0 0 0 0 0 0 0 ]
    //      [ 0 0 0 0 0 0 0 ]
    //      [ 0 0 0 0 0 0 0 ]
    // (A*P + P*A')_ij = A_ik*P_kj + P_ik*A_jk
    
    float AP_plus_PAT[EKF_NUM_STATES][EKF_NUM_STATES] = {0};
    
    for(int i=0; i < 3; i++) { // For pos rows (0-2)
        for(int j=0; j < EKF_NUM_STATES; j++) {
            AP_plus_PAT[i][j] = P(ekf, i+3, j); // A_ik*P_kj part
        }
    }
    
    for(int j=0; j < 3; j++) { // For pos cols (0-2)
        for(int i=0; i < EKF_NUM_STATES; i++) {
            AP_plus_PAT[i][j] += P(ekf, i, j+3); // P_ik*A_jk part
        }
    }

    // Now update P: P = P + dt * (A*P + P*A' + Q)
    for (int i = 0; i < EKF_NUM_STATES; i++) {
        for (int j = 0; j < EKF_NUM_STATES; j++) {
            P(ekf, i, j) += AP_plus_PAT[i][j] * dt;
        }
        // Add process noise Q (diagonal)
        P(ekf, i, i) += ekf->Q[i] * dt;
    }
}

//rather than do complex matrix math we have a more robust method here called sequential update
/**
 * @brief Performs a single-measurement update on the EKF state.
 * This is the standard, robust way to do an EKF update without matrix inversion.
 * @param ekf Pointer to the EKF structure
 * @param innov Innovation (measurement - predicted_state)
 * @param state_index The index of the state being updated (0-6)
 * @param meas_noise The measurement noise (R) for this specific measurement
 */
static void EKF_SequentialUpdate(ParafoilEKF *ekf, float innov, int state_index, float meas_noise) {
    
    // This function implements the sequential Kalman update for a single measurement
    // H_k is a row vector [0 0 ... 1 ... 0] with a 1 at state_index
    
    // 1. Calculate S_k = H_k * P_k * H_k' + R_k
    //    Since H_k is [0...1...0], S_k simplifies to P_kk + R_k
    float S_k = P(ekf, state_index, state_index) + meas_noise;
    
    // Avoid division by zero
    if (S_k < 1e-6f) {
        return;
    }
    float S_k_inv = 1.0f / S_k;

    // 2. Calculate Kalman Gain K_k = P_k * H_k' * S_k_inv
    //    P_k * H_k' simplifies to the k-th column of P
    float K_k[EKF_NUM_STATES];
    for (int i = 0; i < EKF_NUM_STATES; i++) {
        K_k[i] = P(ekf, i, state_index) * S_k_inv;
    }

    // 3. Update state estimate: x_new = x_old + K_k * innov
    ekf->pos_n += K_k[0] * innov;
    ekf->pos_e += K_k[1] * innov;
    ekf->pos_d += K_k[2] * innov;
    ekf->vel_n += K_k[3] * innov;
    ekf->vel_e += K_k[4] * innov;
    ekf->vel_d += K_k[5] * innov;
    ekf->heading_bias += K_k[6] * innov;

    // 4. Update covariance matrix: P_new = (I - K_k * H_k) * P_old
    //    Joseph form, which is more numerically stable (also apparently industry standard)
    float I_minus_KH[EKF_NUM_STATES][EKF_NUM_STATES];
    for (int i = 0; i < EKF_NUM_STATES; i++) {
        for (int j = 0; j < EKF_NUM_STATES; j++) {
            // H_k[j] is 1 only if j == state_index, 0 otherwise
            float KH_ij = (j == state_index) ? K_k[i] : 0.0f;
            
            // I_ij - KH_ij
            I_minus_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH_ij;
        }
    }

    // P_new = (I - K*H) * P_old
    float P_new[EKF_NUM_STATES][EKF_NUM_STATES];
    for (int i = 0; i < EKF_NUM_STATES; i++) {
        for (int j = 0; j < EKF_NUM_STATES; j++) {
            P_new[i][j] = 0.0f;
            for (int k = 0; k < EKF_NUM_STATES; k++) {
                P_new[i][j] += I_minus_KH[i][k] * P(ekf, k, j);
            }
        }
    }
    
    // Copy new P matrix back to ekf structure
    memcpy(ekf->P, P_new, sizeof(ekf->P));
}



/*
 * EKF Update Step - GPS Correction
 * * Runs when GPS data arrives (~1-5 Hz)
 */
int8_t ParafoilEKF_UpdateGPS(ParafoilEKF *ekf,
                             double gps_lat,
                             double gps_lon,
                             float gps_alt,
                             float gps_speed,
                             float gps_heading,
                             float *quat,
                             uint8_t valid_heading) {
    
    // set GPS reference on first fix
    if (!ekf->initialized) {
        ekf->ref_lat = gps_lat;
        ekf->ref_lon = gps_lon;
        ekf->ref_alt = gps_alt;
        ekf->initialized = 1;

        // start at origin for NED
        ekf->pos_n = 0.0f;
        ekf->pos_e = 0.0f;
        ekf->pos_d = 0.0f;

        // Also set velocity from first GPS pulse if valid
        if (valid_heading) {
            float gps_heading_rad = gps_heading * DEG_TO_RAD;
            ekf->vel_n = gps_speed * cosf(gps_heading_rad);
            ekf->vel_e = gps_speed * sinf(gps_heading_rad);
        }
        
        return 0; // First fix, no update yet
    }

    // --- 1. Compute measurement vector z ---
    float gps_pos_n, gps_pos_e, gps_pos_d;
    GPS_ToNED(gps_lat, gps_lon, gps_alt,
              ekf->ref_lat, ekf->ref_lon, ekf->ref_alt,
              &gps_pos_n, &gps_pos_e, &gps_pos_d);

    // gps velocity from speed and heading
    float gps_heading_rad = gps_heading * DEG_TO_RAD;
    float gps_vel_n = 0.0f;
    float gps_vel_e = 0.0f;
    if (valid_heading) {
         gps_vel_n = gps_speed * cosf(gps_heading_rad);
         gps_vel_e = gps_speed * sinf(gps_heading_rad);
    }
    float gps_vel_d = 0.0f; // assume 0 down velocity from GPS

    // get IMU heading from quaternion
    float qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
    float imu_yaw = atan2f(2.0f * (qw * qz + qx * qy),
                          1.0f - 2.0f * (qy * qy + qz * qz));
    
    // --- 2. Compute Innovation Vector (y = z - h(x)) ---
    // h(x) is just x, since we measure the states directly
    
    float innov_pos_n = gps_pos_n - ekf->pos_n;
    float innov_pos_e = gps_pos_e - ekf->pos_e;
    float innov_pos_d = gps_pos_d - ekf->pos_d;
    
    float innov_vel_n = gps_vel_n - ekf->vel_n;
    float innov_vel_e = gps_vel_e - ekf->vel_e;
    float innov_vel_d = gps_vel_d - ekf->vel_d;
    
    float innov_heading = 0.0f;
    if (valid_heading) { 
        float expected_heading = imu_yaw + ekf->heading_bias;
        innov_heading = wrap_to_pi(gps_heading_rad - expected_heading);
    }
    // This updates the state and covariance for each measurement, one by one
    // This is numerically stable and avoids matrix inversion
    
    // R[] indices: 0:pos_n, 1:pos_e, 2:pos_d, 3:vel_n, 4:vel_e, 5:vel_d, 6:heading
    // State indices: 0:pos_n, 1:pos_e, 2:pos_d, 3:vel_n, 4:vel_e, 5:vel_d, 6:heading_bias
    
    EKF_SequentialUpdate(ekf, innov_pos_n, 0, ekf->R[0]);
    EKF_SequentialUpdate(ekf, innov_pos_e, 1, ekf->R[1]);
    EKF_SequentialUpdate(ekf, innov_pos_d, 2, ekf->R[2]);
    
    if (valid_heading) {
        // Only update velocity and heading bias if GPS heading is valid
        EKF_SequentialUpdate(ekf, innov_vel_n, 3, ekf->R[3]);
        EKF_SequentialUpdate(ekf, innov_vel_e, 4, ekf->R[4]);
        EKF_SequentialUpdate(ekf, innov_vel_d, 5, ekf->R[5]);
        EKF_SequentialUpdate(ekf, innov_heading, 6, ekf->R[6]);
    }

    return 0;
}


//Helper functions below

/**
 * get correct heading
 */
float ParafoilEKF_GetHeading(ParafoilEKF *ekf, float *quat) {
    if (!ekf->initialized) return 0.0f;
    
    float qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
    float yaw_rad = atan2f(2.0f*(qw*qz + qx*qy), 
                           1.0f - 2.0f*(qy*qy + qz*qz));
    
    /* Apply bias correction */
    float corrected_yaw = yaw_rad + ekf->heading_bias;
    
    /* Convert to degrees 0-360 */
    float heading_deg = corrected_yaw * RAD_TO_DEG;
    while (heading_deg < 0.0f) heading_deg += 360.0f;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;
    
    return heading_deg;
}

void ParafoilEKF_GetPositionGPS(ParafoilEKF *ekf,
                                double *gps_lat,
                                double *gps_lon,
                                float *gps_alt) {
    if (!ekf->initialized) {
        *gps_lat = 0.0;
        *gps_lon = 0.0;
        *gps_alt = 0.0;
        return;
    }
    
    NED_ToGPS(ekf->pos_n, ekf->pos_e, ekf->pos_d,
              ekf->ref_lat, ekf->ref_lon, ekf->ref_alt,
              gps_lat, gps_lon, gps_alt);
}

void ParafoilEKF_GetVelocity(ParafoilEKF *ekf,
                             float *vel_n,
                             float *vel_e,
                             float *vel_d) {
    if (!ekf->initialized) {
        *vel_n = *vel_e = *vel_d = 0.0f;
        return;
    }
    
    *vel_n = ekf->vel_n;
    *vel_e = ekf->vel_e;
    *vel_d = ekf->vel_d;
}

// is parafoil ready
uint8_t ParafoilEKF_IsReady(ParafoilEKF *ekf) {
    return ekf->initialized;
}



