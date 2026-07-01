#ifndef BALANCE_TASK_H
#define BALANCE_TASK_H
#include "main.h"

typedef struct{
float expect_x,expect_speed;
float L0,L1;
float location_x;
float display_roll_l,display_roll_r;
float output_l_speed,output_l_pitch,output_l_gyro,output_l_position;
float output_all_l[2];
float theta,d_theta;
float real_speed,real_location;
u8 L0_station;
}balance_leg;

typedef struct{
float legl0;
float theta,w_theta;
float location,speed;
float pitch,w_pitch;
float target_speed;	
float T_O[2];
float f_support;
float T_K_[2][6];
}balance_usedata;

void balance_task();


#endif
