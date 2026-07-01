#ifndef __UNITREEMOTOR_H
#define __UNITREEMOTOR_H

#include "motor_msg_GO-M8010-6.h" 
#include <stdint.h>

    typedef enum
    {
        A1,    // 4.8M baudrate
        B1_16, // 6.0M baudrate
        GO_M8010_6
    } MotorType;

    typedef enum
    {
        BRAKE,
        FOC,
        CALIBRATE
    } MotorMode;

    /*******************************************************************
     * @brief  电机期望值设定，用于设定电机的期望值 des = desired value
     * @param mode,ID,tor_des,spd_des,pos_des,kp,k_spd
     ******************************************************************/
    typedef struct
    {
        MotorType motorType;
        uint8_t ID;     // 电机编号，可以为 0、1、2、0xBB，0xBB代表向所有电机广播
        MotorMode mode; // 运行模式
        float tor_des;  // 期望关节输出扭矩 unit: N.m     (q8)
        float spd_des;  // 期望关节输出速度 unit: rad/s   (q7)
        float pos_des;  // 期望关节输出位置 unit: rad     (q15)
        float kp_des;   // 期望关节刚度系数 unit: 0.0-1.0 (q15)
        float kd_des;   // 期望关节阻尼系数 unit: 0.0-1.0 (q15)
    } MotorSetValTypeDef;

// Utility Function
int queryMotorMode(MotorType motortype,MotorMode motormode);
float queryGearRatio(MotorType motortype);
#endif  // UNITREEMOTOR_H
		
		
		

		