/*
 * KalmanFilter.h
 *
 *  Created on: Feb 18, 2024
 *      Author: Luka
 */

#ifndef INC_GAUL_DRIVERS_KALMANFILTER_H_
#define INC_GAUL_DRIVERS_KALMANFILTER_H_


#include "GAUL_Drivers/ICM20602.h"
void kalman1D(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurment);
void getRollPitch(ICM20602 *dev);



#endif /* INC_GAUL_DRIVERS_KALMANFILTER_H_ */



