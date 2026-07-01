#ifndef __SPEED_KALMAN_TASK_H
#define __SPEED_KALMAN_TASK_H

#include "kalman_filter.h"

void speed_kalman_task();
void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);

#endif




