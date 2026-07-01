#ifndef LQR_H
#define LQR_H
#include "main.h"

void lqr_set_matrix_K(LQR* obj,float* matrix_k);
void lqr_calc(LQR*obj);

#endif
