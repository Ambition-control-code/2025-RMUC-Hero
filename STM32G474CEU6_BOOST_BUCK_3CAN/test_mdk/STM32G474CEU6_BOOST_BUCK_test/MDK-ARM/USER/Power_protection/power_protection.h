#ifndef _pow_protection_
#define _pow_protection_

#include "main.h"


//保护参数限制
#define  CAP_V_limit     32000   //电容电压限制 mv
#define  CAP_I_limit     17000   //电容电流限制 ma
#define  POWER_V_limit   32000  //电源电压限制 mv
#define  POWER_I_limit   17000   //电源电流限制 ma

#define ON 1
#define OFF 0

extern uint8_t HRTIM_BIT; //HRTIM定时器状态标志 0关闭，1开启

uint8_t POWER_PROTECTION(void);
void HRTIM_OFF(void);
void HRTIM_ON(void);
void DEBUG_TETST_SHOW(void);
void U_I_CTRL_INIT(void);
float abs_user(float in);



#endif

