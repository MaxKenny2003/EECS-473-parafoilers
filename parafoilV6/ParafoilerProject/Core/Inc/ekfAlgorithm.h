/*
 * ekfAlgorithm.h
 *
 *  Created on: Oct 24, 2025
 *      Author: tonym
 */

#ifndef INC_EKFALGORITHM_H_
#define INC_EKFALGORITHM_H_


/* Includes ------------------------------------------------------------------*/
//#include "stm32h7xx_hal.h"
//#include "BNO085_SPI_Library.h"
//#include "Hardware_Init.h"
#include <math.h>

/* Private Define */
//#define EKF_PREDICT_PERIODMS 10
//#define EKF_UPDATE_PERIODMS 100

//#define gyroScopeAlpha 0.01
//#define magAlpha 0.01
//#define accelAlpha 0.01
//#define gpsAlpha 0.01
//#define gpsAltAlpha 0.01

// Define covarance initial values
#define KalPInit 0.1f
#define KalQInit 0.001f
#define KalRInit 0.011f


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
static float x[12];

// Matrices for Covariances, Gravity Constant, Sampling Time(s)
// Matrix size for Github Version
//static float P[144];
//static float Q[144];
//static float R[36];
// Matrix size for youtube video
static float P[4];
static float Q[3];
static float R[2];
static float g = 9.81;
static float T = 0.01;

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
	T = 0.01; // Assume 10ms period for grabbing sensor data

	// Initialize State Estimate / Measurement Variables
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

// To-Do: Add extra arguments for reading in values if necessary
// To-Do 2: If it becomes too computationally expensive to calculate sinusoids, discuss with team to make a look-up table to trade off memory for execution speed

// Phil's Lab Youtube Init

static ekfAlgoInitTwo(ekfVariables* structPtr, float Pinit, float Qinit, float Rinit){
	structPtr->gyroPitch = 0.0;
	structPtr->gyroRoll = 0.0;

	// Initialize Covariance Matrices
	P[0] = Pinit; P[1] = 0.0f;
	P[2] = 0.0f;  P[3] = Pinit;

	Q[0] = Qinit;
	Q[1] = Qinit;

	R[0] = Rinit;
	R[1] = Rinit;
	R[2] = Rinit;
}
static void ekfAlgoPredict(ekfVariables* structPtr, float imuSensorValues[12]){
	// Capture data from our sensors, filter them, and put it into temp variables

	float phiRadians = imuSensorValues[3]*alphaGyr + (1.0 - alphaGyr)*imuSensorValues[3];
	float thetaRadians = imuSensorValues[4]*alphaGyr + (1.0 - alphaGyr)*imuSensorValues[4];
	float psiRadians = imuSensorValues[5]*alphaGyr + (1.0 - alphaGyr)*imuSensorValues[5];
//	structPtr->accelX = imuSensorValues[0];
//	structPtr->accelY = imuSensorValues[1];
//	structPtr->accelZ = imuSensorValues[2];
//    structPtr->gyroRoll = imuSensorValues[3];
//    structPtr->gyroPitch = imuSensorValues[4];
//    structPtr->gyroYaw = imuSensorValues[5];
//    structPtr->magX = imuSensorValues[6];
//    structPtr->magY = imuSensorValues[7];
//    structPtr->magZ = imuSensorValues[8];
//    structPtr->gpsLat = imuSensorValues[9];
//    structPtr->gpsLong = imuSensorValues[10];
//    structPtr->gpsAlt = imuSensorValues[11];
    	//TO-DO!!! Update this function to include GPS variables and other possible sensor data


	// Next, perform the prediction calculation
	// Gyroscope: Calculate common trig terms
	// Phi = Roll = p
	// Theta = Pitch = q
	// Psi = Yaw = r
	float sinPhi = sin(phiRadians);
	float cosPhi = cos(phiRadians);
	float tanTheta = tan(thetaRadians);
	// ^^ Remember from trig that tan(a) = sin(a) / cos(a)
	// Gyroscope: Forward Euler Intergrate to solve for state variable
	structPtr->gyroRoll = structPtr->gyroRoll + T*(phiRadians + tanTheta*((thetaRadians*sinPhi) + (psiRadians*cosPhi)));
	structPtr->gyroPitch = structPtr->gyroPitch + T*((thetaRadians*cosPhi) - (psiRadians*sinPhi));
//	structPtr->gyroRoll

	// Gyroscope: Recalculate trig terms w/ the new State Estimate Variables
	sinPhi = sin(structPtr->gyroRoll);
	cosPhi = cos(structPtr->gyroRoll);
	float sinTheta = sin(structPtr->gyroPitch);
	float cosTheta = cos(structPtr->gyroPitch);
	tanTheta = sinTheta / cosTheta;

	// Gyroscope: Calculate the Jacobian Matrix of the function f(x, u)
	float gyroA[4] = {
			tanTheta*(thetaRadians*cosPhi - psiRadians*sinPhi), // A[0]
			thetaRadians*sinPhi*(tanTheta*tanTheta + 1.0f) + psiRadians*cosPhi*(tanTheta*tanTheta + 1.0f), // A[1]
			-(thetaRadians*sinPhi+psiRadians*cosPhi), // A[2]
			0.0f // A[3]
	};

	// Gyroscope:Recalculate the Covariance Matrix

	float Ptmp[4] = {
		T*(Q[0] + 2.0f * gyroA[0]*P[0] + gyroA[1]*P[1] + gyroA[1]*P[2]),
		T*(gyroA[0]*P[1] + gyroA[2]*P[0] + gyroA[1]*P[3] + gyroA[3]*P[1]),
		T*(gyroA[0]*P[2] + gyroA[2]*P[0] + gyroA[1]*P[3] + gyroA[3]*P[2]),
		T*(Q[1] + gyroA[2]*P[1] + gyroA[2]*P[2] + 2.0f*gyroA[3]*P[3])
	};

	// Update Covariance Matrix with these new values
	P[0] = P[0] + Ptmp[0];
	P[1] = P[1] + Ptmp[1];
	P[2] = P[2] + Ptmp[2];
	P[3] = P[3] + Ptmp[3];
}

static void ekfAlgoUpdate(ekfVariables* structPtr, float imuSensorValues[12]){
	// Accelerometer: Read sensor values
	float accelXRead = imuSensorValues[0]*alphaAcc + (1.0 - alphaAcc)*imuSensorValues[0];
	float accelYRead = imuSensorValues[1]*alphaAcc + (1.0 - alphaAcc)*imuSensorValues[1];
	float accelZRead = imuSensorValues[2]*alphaAcc + (1.0 - alphaAcc)*imuSensorValues[2];

	// Gyroscope: Calculate Trig Functions
	float sinPhi = sin(structPtr->gyroRoll);
	float cosPhi = cos(structPtr->gyroRoll);
	float sinTheta = sin(structPtr->gyroPitch);
	float cosTheta = cos(structPtr->gyroPitch);

	// Accelerometer: Calculate the Output Function, h(x,u)
	float h[3] = {
			g*sinTheta,
			-g*cosTheta*sinPhi,
			-g*cosTheta*cosPhi
	};

	// Accelerometer: Calculate Matrix C, the Jacobian of h(x,u)
	float C[6] = {
		0,
		g*cosTheta,
		-g*cosTheta*cosPhi,
		-g*sinTheta*sinPhi,
		g*cosTheta*sinPhi,
		g*sinTheta*cosPhi
	};
	// Accelerometer: Kalman Gain : K = P * CT / (C*P*CT + R)^-1
	// Use all variables in C to make this form reuseable for later
	float G[9] = {
			R[0] + C[0]*(C[0]*P[0] + C[1]*P[2]) + C[1]*(C[0]*P[1] + C[1]*P[3]),
			C[2]*(C[0]*P[0] + C[1]*P[2]) + C[3]*(C[0]*P[1] + C[1]*P[3]),
			C[4]*(C[0]*P[0] + C[1]*P[2]) + C[5]*(C[0]*P[1] + C[1]*P[3]),
			C[0]*(C[2]*P[0] + C[3]*P[2]) + C[1]*(C[2]*P[1] + C[3]*P[3]),
			R[1] + C[2]*(C[2]*P[0] + C[3]*P[2]) + C[3]*(C[2]*P[1] + C[3]*P[3]),
			C[4]*(C[2]*P[0] + C[3]*P[2]) + C[5]*(C[2]*P[1] + C[3]*P[3]),
			C[0]*(C[4]*P[0] + C[5]*P[2]) + C[1]*(C[4]*P[1] + C[5]*P[3]),
			C[2]*(C[4]*P[0] + C[5]*P[2]) + C[3]*(C[4]*P[1] + C[5]*P[3]),
			R[2] + C[4]*(C[4]*P[0] + C[5]*P[2]) + C[5]*(C[4]*P[1] + C[5]*P[3])
	};

	// Calculate the matrix G determinant AND take its inverse:
	float GDetInv = 1.0f / (G[0]*G[4]*G[8] - G[0]*G[5]*G[7] - G[1]*G[3]*G[8] + G[1]*G[5]*G[6] + G[2]*G[3]*G[7] - G[2]*G[4]*G[6]);

	/*
	 * [  G4*G8 - G5*G7,  -(G1*G8 - G2*G7),   G1*G5 - G2*G4;
  -(G3*G8 - G5*G6),  G0*G8 - G2*G6,  -(G0*G5 - G2*G3);
   G3*G7 - G4*G6,  -(G0*G7 - G1*G6),   G0*G4 - G1*G3 ]
	 * */
	// This is the Adjugate of G divided by the determinant of G
	float GInv[9] = {
		GDetInv*(G[4]*G[8] - G[5]*G[7]),
		-GDetInv*(G[1]*G[8] - G[2]*G[7]),
		GDetInv*(G[1]*G[5] - G[2]*G[4]),
		-GDetInv*(G[3]*G[8]-G[5]*G[6]),
		GDetInv*(G[0]*G[8]-G[2]*G[6]),
		-GDetInv*(G[0]*G[5]-G[2]*G[3]),
		GDetInv*(G[3]*G[7]-G[4]*G[6]),
		-GDetInv*(G[0]*G[7]-G[1]*G[6]),
		GDetInv*(G[0]*G[4]-G[1]*G[3])
	};

	float K[6] = {
			GInv[0]*(C[0]*P[0]+C[1]*P[1]) + GInv[3]*(C[2]*P[0]+C[3]*P[1]) + GInv[6]*(C[4]*P[0]+C[5]*P[1]),
			GInv[1]*(C[0]*P[0] + C[1]*P[1]) + GInv[4]*(C[2]*P[0] + C[3]*P[1]) + GInv[7]*(C[4]*P[0] + C[5]*P[1]),
			GInv[2]*(C[0]*P[0] + C[1]*P[1]) + GInv[5]*(C[2]*P[0] + C[3]*P[1]) + GInv[8]*(C[4]*P[0] + C[5]*P[1]),
			GInv[0]*(C[0]*P[2] + C[1]*P[3]) + GInv[3]*(C[2]*P[2] + C[3]*P[3]) + GInv[6]*(C[4]*P[2] + C[5]*P[3]),
			GInv[1]*(C[0]*P[2] + C[1]*P[3]) + GInv[4]*(C[2]*P[2] + C[3]*P[3]) + GInv[7]*(C[4]*P[2] + C[5]*P[3]),
			GInv[2]*(C[0]*P[2] + C[1]*P[3]) + GInv[5]*(C[2]*P[2] + C[3]*P[3]) + GInv[8]*(C[4]*P[2] + C[5]*P[3])
	};

	/* My MATLAB Code:
	 * PTmp =
[- P0*(C0*K0 + C2*K1 + C4*K2 - 1) - P2*(C1*K0 + C3*K1 + C5*K2), - P1*(C0*K0 + C2*K1 + C4*K2 - 1) - P3*(C1*K0 + C3*K1 + C5*K2)]
[- P0*(C0*K3 + C2*K4 + C4*K5) - P2*(C1*K3 + C3*K4 + C5*K5 - 1), - P1*(C0*K3 + C2*K4 + C4*K5) - P3*(C1*K3 + C3*K4 + C5*K5 - 1)]
	 * */

	// Phils code
	/*
	 *
		float Ptmp[4];
		Ptmp[0] = -kal->P[2]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - kal->P[0]*(C[2]*K[1] + C[4]*K[2] - 1.0f);
		Ptmp[1] = -kal->P[3]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - kal->P[1]*(C[2]*K[1] + C[4]*K[2] - 1.0f);
		Ptmp[2] = -kal->P[2]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5]) - kal->P[0]*(C[2]*K[4] + C[4]*K[5]);
		Ptmp[3] = -kal->P[3]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5]) - kal->P[1]*(C[2]*K[4] + C[4]*K[5]);

		kal->P[0] = kal->P[0] + Ptmp[0];
		kal->P[1] = kal->P[1] + Ptmp[1];
		kal->P[2] = kal->P[2] + Ptmp[2];
		kal->P[3] = kal->P[3] + Ptmp[3];
	 *
	 * */
	// Define temporary step for updating matrix P
	// to-do: Take a look at this matrix. Perhaps the math is wrong?
	float Ptmp[4];
	Ptmp[0] = -P[2]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - P[0]*(C[2]*K[1] + C[4]*K[2] - 1.0f);
	Ptmp[1] = -P[3]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - P[1]*(C[0]*K[0] + C[2]*K[1] + C[4]*K[2] - 1.0f);
	Ptmp[2] = -P[0]*(C[0]*K[3] + C[2]*K[4] + C[4]*K[5]) - P[2]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f);
	Ptmp[3] = -P[1]*(C[0]*K[3] + C[2]*K[4] + C[4]*K[5]) - P[3]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f);

	// Update covariance matrix P:
	P[0] = P[0] + Ptmp[0];
	P[1] = P[1] + Ptmp[1];
	P[2] = P[2] + Ptmp[2];
	P[3] = P[3] + Ptmp[3];

	// Update State Estimate: (From Phil's Video)
	structPtr->gyroRoll = structPtr->gyroRoll + K[0] * (accelXRead - h[0]) + K[1] * (accelYRead - h[1]) + K[2] * (accelZRead - h[2]);
	structPtr->gyroPitch = structPtr->gyroPitch + K[3] * (accelXRead - h[0]) + K[4] * (accelYRead - h[1]) + K[5] * (accelZRead - h[2]);
}


#endif /* INC_EKFALGORITHM_H_ */
