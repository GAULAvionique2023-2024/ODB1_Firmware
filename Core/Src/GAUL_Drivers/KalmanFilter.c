/*
 * KalmanFilter.c
 *
 *  Created on: Feb 18, 2024
 *      Author: Luka
 */

#include "GAUL_Drivers/KalmanFilter.h"

float kalmanUncertaintyAngleRoll = 		2*2;
float kalmanUncertaintyAnglePitch =		2*2;

float kalman1DOutput[] = 				{0,0};

void kalman1D(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurment)
{
	kalmanState += kalmanInput*0.004;
	kalmanUncertainty += 0.004 * 0.004 * 4 * 4;
	float kalmanGain = kalmanUncertainty * 1 / (1 * kalmanUncertainty + 3 * 3);
	kalmanState += kalmanGain * (kalmanMeasurment - kalmanState);
	kalmanUncertainty = (1-kalmanGain) * kalmanUncertainty;

	kalman1DOutput[0] = kalmanState;
	kalman1DOutput[1] = kalmanUncertainty;
}

void getRollPitch(ICM20602 *dev)
{
	  kalman1D(dev->kalmanAngleRoll, kalmanUncertaintyAngleRoll, dev->gyroX, dev->angleRoll);
	  dev->kalmanAngleRoll = kalman1DOutput[0];
	  kalmanUncertaintyAngleRoll = kalman1DOutput[1];
	  kalman1D(dev->kalmanAnglePitch, kalmanUncertaintyAnglePitch, dev->gyroY, dev->anglePitch);
	  dev->kalmanAnglePitch = kalman1DOutput[0];
	  kalmanUncertaintyAnglePitch = kalman1DOutput[1];
}
