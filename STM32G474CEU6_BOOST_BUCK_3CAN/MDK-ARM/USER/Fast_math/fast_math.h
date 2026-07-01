#ifndef _FAST_MATH_H_
#define _FAST_MATH_H_
#include "main.h"
//使用STM32G474硬件Cordic单元加速三角函数运算


void LPF_Init(LOW_Pass_Filter *LPF, float threshold);
float LPF_add(LOW_Pass_Filter *LPF, float input_data);

//快速开根号
float InvSqrt(float x);
//查表法求cos
float FindList_COS(float in);

void Calculate_Float_Sin_Cos(float angle,float *sin, float *cos);

void Calculate_Float_arctan_all(float x,float y,float *angle);
#endif




