/*
 * ekfAlgo.h
 *
 *  Created on: Oct 21, 2025
 *      Author: tonym
 */

#ifndef INC_EKFALGO_H_
#define INC_EKFALGO_H_

/* Private Define */
#define EKF_PREDICT_PERIODMS 10
#define EKF_UPDATE_PERIODMS 100

/* Variable Definitions */

// Gyroscope Variables
static double eulerP;
static double eulerQ;
static double eulerR;
// Accelerometer Variables
static double accelX;
static double accelY;
static double accelZ;
// Magnitometer Variables
static double magX;
static double magY;
static double magZ;
// LPF Variables
static double lpfGyr;
static double lpfAcc;
static double lpfMag;

// Matrices for Covariances and Gravity Constant
static double P[49];
static double Q[49];
static double R[36];
static double g;

// GPS
static double gpsLat;
static double gpsLong;

// Functions

// To-do: Add GPS noise to the argument list and incorporate it into the covariance calculations
static void ekfAlgoInit(const double noiseGyro, const double noiseAccel, const double noiseMag){
	// State Covariance: "How much do we trust our initial state estimates?"
	static const double dv4[7] = { 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-8, 1.0E-8,
	1.0E-8 };
	// Noise Covariance: "How much random variation do we expect from the dynamics / gyroscope drift per step"
	static const double dv5[7] = { nGyro * nGyro, nGyro * nGyro, nGyro * nGyro, nGyro * nGyro, 1.0E-9,
	1.0E-9, 1.0E-9 };
	// Measurement Noise Covariance: "How much uncertainty from the sensor measurements do we have?"
	static const double dv6[6] = { nAcc * nAcc, nAcc * nAcc,
	nAcc * nAcc, nMag, nMag, nMag };

	// Initialize Gravitational Constant
	g = 9.81;

	// Initialize LPFed Measurement Variables
	eulerP = 0.0;
	eulerQ = 0.0;
	eulerR = 0.0;
	accelX = 0.0;
	accelY = 0.0;
	accelZ = 0.0;
	magX   = 0.0;
	magY   = 0.0;
	magZ   = 0.0;

	// Initialize the Bias / Calibration Bias
	// To-do: Recalibrate this in the context of our project
	lpfGyr = 0.7;
	lpfAcc = 0.9;
	lpfMag = 0.4;
	lpfVa = 0.7;

	// Initialize each vector / matrix
	int iter = 0;
	for(iter = 0; iter < 7; iter++){
		x[i] = 0.0;
	}
	x[0] = 1.0;

	/* Initialize Covariance Matrix*/
	memset(&P[0], 0, 49U * sizeof(double));
	for (iter = 0; iter < 7; iter++) {
		P[iter + 7 * iter] = dv4[iter];
	}
	/*	Initialize Noise Matrix	*/
	memset(&Q[0], 0, 49U * sizeof(double));
	for (iter = 0; iter < 7; iter++) {
		Q[iter + 7 * iter] = dv5[iter];
	}

	/* Measurement Noise Matrix	*/
	memset(&R[0], 0, 36U * sizeof(double));
	for (iter = 0; iter < 6; iter++) {
		R[iter + 6 * iter] = dv6[iter];
	}
}

#endif /* INC_EKFALGO_H_ */
