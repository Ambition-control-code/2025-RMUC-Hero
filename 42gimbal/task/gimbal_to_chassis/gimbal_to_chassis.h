#ifndef GIMBAL_TO_CHASSIS_H
#define GIMBAL_TO_CHASSIS_H
#include "main.h"

typedef union{
u8 rc_station;
struct{
u8   zimiao_live:1;
u8	 direction:3;
u8    rc_sr : 2;
u8    rc_sl : 2;
}rc;
}pack_send_gimbal;

typedef union{
u8    keyboard;
struct{
u8    f      :1;
u8	  e      :1;
u8    shift  :1;
u8    q      :1;
u8    r      :1;
u8	  c      :1;
u8    mouse_r:1;
u8    mouse_l:1;
}rc;
}pack_send_mouse;

void gimbal_to_chassis_task(void);


#endif
