#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "main.h"

typedef struct{
float pitch_zimiao,yaw_zimiao;
	
}gimbal_message;

typedef struct{
float pitch_shoucao,yaw_shoucao;
float pitch_zimiao,yaw_zimiao;
float yaw_speed;
float fire_speed,PID_HERO_CLIP_ANGLE_sum,last_sum;
char zimiao_use,Dial_behavior,enable_dial,auto_fire,last_auto_fire;
int16_t now_hot,max_hot;
}Gimbal_all;

void gimbal_init();
void gimbal_task();


#endif
