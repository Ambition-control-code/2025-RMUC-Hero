#ifndef _pow_protection_
#define _pow_protection_

#include "main.h"


//保护参数限制
#define  CAP_V_limit     37000   //电容电压限制 mv
#define  CAP_I_limit     25000   //电容电流限制 ma
#define  POWER_V_limit   37000   //电源电压限制 mv
#define  POWER_I_limit   15000   //电源电流限制 ma
#define  ADC_V_MAX       45000   //最大电压限制 mv

#define ON  1
#define OFF 0

extern uint8_t HRTIM_BIT; //HRTIM定时器状态标志 0关闭，1开启

uint8_t POWER_PROTECTION(void);
 uint8_t POWER_PROTECTION_LITE(void);
void HRTIM_OFF(void);
void HRTIM_ON(void);
void DEBUG_TETST_SHOW(void);
void U_I_CTRL_INIT(void);
float abs_user(float in);

	typedef struct
{
	uint8_t CAP_LIM_I_STATE;     //电容超流标志
	uint8_t CAP_LIM_U_STATE;     //电容超压标志
  uint8_t POW_LIM_I_STATE;     //底盘超流标志
	uint8_t POW_LIM_U_STATE;     //底盘超压标志
	uint8_t CHASSIS_POWER_STATE; //裁判系统供电标志
	uint8_t LOOP_EFF_STATE;      //工作效率标志
	uint8_t CAP_ERR_STATE;       //电容错误标志
}Power_err;
extern Power_err Power_error_state;

#endif

