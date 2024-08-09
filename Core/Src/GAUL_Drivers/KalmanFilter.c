/*
 * KalmanFilter.c
 *
 *  Created on: Feb 18, 2024
 *      Author: Luka
 */

#include "GAUL_Drivers/KalmanFilter.h"

void KalmanFilter_Init(KalmanFilter *kf)
{
    kf->Q_angle = Q_ANGLE;
    kf->Q_bias = Q_BIAS;
    kf->R_measure = R_MEASURE;

    kf->angle = 0.0f;
    kf->bias = 0.0f;

    kf->dt = 0;
    kf->kt = HAL_GetTick();

    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

double KalmanFilter_Update(KalmanFilter *kf, double newAngle, double newRate)
{
	kf->dt = ((double)HAL_GetTick() - kf->kt) / 1000;

	kf->rate = newRate - kf->bias;
	kf->angle += kf->dt * kf->rate;

	kf->P[0][0] += kf->dt * (kf->P[1][1] + kf->P[0][1]) + kf->Q_angle * kf->dt;
	kf->P[0][1] -= kf->dt * kf->P[1][1];
	kf->P[1][0] -= kf->dt * kf->P[1][1];
	kf->P[1][1] += kf->Q_bias * kf->dt;

	kf->S = kf->P[0][0] + kf->R_measure;

	kf->K[0] = kf->P[0][0] / kf->S;
	kf->K[1] = kf->P[1][0] / kf->S;

	kf->y = newAngle - kf->angle;

	kf->angle += kf->K[0] * kf->y;
	kf->bias += kf->K[1] * kf->y;

	kf->P[0][0] -= kf->K[0] * kf->P[0][0];
	kf->P[0][1] -= kf->K[0] * kf->P[0][1];
	kf->P[1][0] -= kf->K[1] * kf->P[0][0];
	kf->P[1][1] -= kf->K[1] * kf->P[0][1];

	kf->kt = HAL_GetTick();

	return kf->angle;
}

