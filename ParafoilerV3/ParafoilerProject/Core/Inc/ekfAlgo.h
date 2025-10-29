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

/* Private Define */
#define EKF_PREDICT_PERIODMS 10
#define EKF_UPDATE_PERIODMS 100

#define gyroScopeAlpha 0.01
#define magAlpha 0.01
#define accelAlpha 0.01
#define gpsAlpha 0.01
#define gpsAltAlpha 0.01


/* Variable Definitions */
typedef struct {
	// Gyroscope Variables
	static float gyroRoll;
	static float gyroPitch;
	static float gyroYaw;

	// Accelerometer Variables
	static float accelX;
	static float accelY;
	static float accelZ;

	// Magnitometer Variables
	static float magX;
	static float magY;
	static float magZ;

	// GPS
	static float gpsLat;
	static float gpsLong;
	static float gpsAlt;

} ekfVariables;

// LPF Variables
static float alphaGyr;
static float alphaAcc;
static float alphaMag;
static float alphaGPS;

// State Vector
// 12 Variables, one for each measurement taken by the sensors
static double x[12];

// Matrices for Covariances and Gravity Constant
static float P[144];
static float Q[144];
static float R[36];
static float g;


// Functions

// To-do: Add GPS noise to the argument list and incorporate it into the covariance calculations
static void ekfAlgoInit(ekfVariables* structPtr, const float noiseGyro, const float noiseAccel, const float noiseMag, const float noiseGPS){
// Initialization Values for matrices P, Q and R

//	// State Covariance: "How much do we trust our initial state estimates?"
	static const float PInitArr[12] = { 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-8, 1.0E-8, 1.0E-8, 1.0E-8, 1.0E-8, 1.0E-8, 1.0E-8, 1.0E-8};
//	// Noise Covariance: "How much random variation do we expect from the dynamics / gyroscope drift per step"
//	static const float QInitArr[7] = { noiseGyro * noiseGyro, noiseGyro * noiseGyro, noiseGyro * noiseGyro, noiseGyro * noiseGyro, 1.0E-9, 1.0E-9, 1.0E-9 };
//	// Measurement Noise Covariance: "How much uncertainty from the sensor measurements do we have?"
//	static const float RInitArr[6] = { nAcc * nAcc, nAcc * nAcc,
//	nAcc * nAcc, nMag, nMag, nMag };

    // Initialize Gravitational Constant
	g = 9.81;

	// Initialize Measurement Variables
	structPtr->accelX = 0.0;
	structPtr->accelY = 0.0;
	structPtr->accelZ = 0.0;
    structPtr->gyroRoll = 0.0;
    structPtr->gyroPitch = 0.0;
    structPtr->gyroYaw = 0.0;
    structPtr->magX = 0.0;
    structPtr->magY = 0.0;
    structPtr->magZ = 0.0;
    structPtr->gpsLat = 0.0;
    structPtr->gpsLong = 0.0;
    structPtr->gpsAlt = 0.0;

	// Initialize the Bias / Calibration Bias
	// To-do: Recalibrate this in the context of our project
	alphaGyr = 0.7;
	alphaAcc = 0.9;
	alphaMag = 0.4;
    // lpfGPS = ???

	// Initialize each state variable matrix with their initial values:
	int iter = 0;
	int iterTwo = 0;
	// P-Matrix
	for(iter = 0; iter < 12; iter++){
		for(iterTwo = 0; iterTwo < 12; iterTwo++){
			//	Define all values along the diagonal to be their value from P Array
			if(iterTwo == iter){
				P[iterTwo + 12*iter] = PInitArr[iter];
			}
			else{
				P[iterTwo + 12*iter] = 0.0f;
			}
			//	Otherwise, define all values not along the diagonal to be 0.0
		}
	}

	// Q-Matrix
    //	for(iter = 0; iter < 7; iter++){
        //		x[i] = 0.0;
        //	}
        //	x[0] = 1.0;

        //	/* Initialize Covariance Matrix*/
        //	memset(&P[0], 0, 49U * sizeof(double));
        //	for (iter = 0; iter < 7; iter++) {
            //		P[iter + 7 * iter] = dv4[iter];
            //	}
	// R-Matrix
	//	/*	Initialize Noise Matrix	*/
	//	memset(&Q[0], 0, 49U * sizeof(double));
	//	for (iter = 0; iter < 7; iter++) {
		//		Q[iter + 7 * iter] = dv5[iter];
		//	}
		//
		//	/* Measurement Noise Matrix	*/
		//	memset(&R[0], 0, 36U * sizeof(double));
		//	for (iter = 0; iter < 6; iter++) {
			//		R[iter + 6 * iter] = dv6[iter];
			//	}
}

static void ekfAlgoParseAndFilterData(ekfVariables* structPtr, float imuSensorValues[12]){
	// Capture data from our sensors and put it into our struct
	structPtr->accelX = imuSensorValues[0];
	structPtr->accelY = imuSensorValues[1];
	structPtr->accelZ = imuSensorValues[2];
    structPtr->gyroRoll = imuSensorValues[3];
    structPtr->gyroPitch = imuSensorValues[4];
    structPtr->gyroYaw = imuSensorValues[5];
    structPtr->magX = imuSensorValues[6];
    structPtr->magY = imuSensorValues[7];
    structPtr->magZ = imuSensorValues[8];
//    structPtr->gpsLat = imuSensorValues[9];
//    structPtr->gpsLong = imuSensorValues[10];
//    structPtr->gpsAlt = imuSensorValues[11];

    // Perform Filtering on our sensors
    structPtr->accelX = structPtr->accelX*alphaAcc + (1.0 - alphaAcc)*structPtr->accelX;
    structPtr->accelY = structPtr->accelY*alphaAcc + (1.0 - alphaAcc)*structPtr->accelY;
    structPtr->accelZ = structPtr->accelZ*alphaAcc + (1.0 - alphaAcc)*structPtr->accelZ;
    structPtr->gyroRoll = structPtr->gyroRoll*alphaGyr + (1.0 - alphaGyr)*structPtr->gyroRoll;
    structPtr->gyroPitch = structPtr->gyroPitch*alphaGyr + (1.0 - alphaGyr)*structPtr->gyroPitch;
    structPtr->gyroYaw = structPtr->gyroYaw*alphaGyr + (1.0 - alphaGyr)*structPtr->gyroYaw;
    structPtr->magX = structPtr->magX*alphaMag + (1.0 - alphaMag)*structPtr->magX;
    structPtr->magY = structPtr->magY*alphaMag + (1.0 - alphaMag)*structPtr->magY;
    structPtr->magZ = structPtr->magZ*alphaMag + (1.0 - alphaMag)*structPtr->magZ;

    	//TO-DO!!! Update this function to include GPS variables and other possible sensor data
	
}

// To-Do: Add extra arguments for reading in values if necessary
// To-Do 2: If it becomes too computationally expensive to calculate sinusoids, discuss with team to make a look-up table to trade off memory for execution speed
static void ekfAlgoPredict(ekfVariables* structPtr){
	
	// Calculate each state transition function result f(x,u)


}

#endif /* INC_EKFALGO_H_ */
