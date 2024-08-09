/*
 * KalmanFilter.h
 *
 *  Created on: Feb 18, 2024
 *      Author: Luka
 */
#ifndef INC_GAUL_DRIVERS_KALMANFILTER_H_
#define INC_GAUL_DRIVERS_KALMANFILTER_H_

#include "stm32f1xx_hal.h"
#include "util.h"

#define Q_ANGLE 	0.001f
#define Q_BIAS  	0.003f
#define R_MEASURE  	0.03f

typedef struct {
	double Q_angle;
	double Q_bias;
	double R_measure;

	double angle;
	double bias;
	double rate;

    double S, y;
    double dt,kt;

    double P[2][2];
    double K[2];
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *kf);
double KalmanFilter_Update(KalmanFilter *kf, double newAngle, double newRate);

#endif /* INC_GAUL_DRIVERS_KALMANFILTER_H_ */

