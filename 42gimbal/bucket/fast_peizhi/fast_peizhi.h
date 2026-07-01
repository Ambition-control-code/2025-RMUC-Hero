#ifndef ulitility_tools_H
#define ulitility_tools_H
#include "main.h"

void  judege_station(int16_t a,int16_t b,Gimbal_hero *HERO_AMB);
float gimbal_pitch_replay(float now_pitch,float *a);
void  OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);


#endif
