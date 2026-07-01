#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H
#include "main.h"

void Slope_Init(slope* Slope,int slope_k);
int Slope_use(slope* Slope,int in);
void power_limit(float pow_max, double k1, double k2,float *speed_3508,float *power,float *use_power);


#endif
