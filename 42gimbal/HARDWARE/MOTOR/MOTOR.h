#ifndef _motor_H
#define _motor_H

#include "main.h"  

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "unitreeMotor.h"
#include "gimbal_to_chassis.h"

#pragma pack(1) // 指令来设置结构体的对齐方式!!!!!!!!!!!!非常重要

//    typedef struct
//    {
//		
//    } HEAD_mode_set;           // 2+1bytes

//    typedef struct 
//    {
//        /* data */

//    } MasterComd;        // 共计12bytes,加上数据包的包头 和CRC 17字节

typedef struct
{
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 电机控制模式  1Byte
    RIS_Fbk_t   fbk;    // 电机反馈数据 11Byte
    uint16_t  CRC16;    // CRC          2Byte

} RecvData_t;      // 电机返回数据     16Byte


    typedef struct
    {
      uint8_t s1        :8;    // 1数据包头1 固定值为0xFE，2数据包头2 固定值为0xEE
      uint8_t s2        :8;    // 1数据包头1 固定值为0xFE，2数据包头2 固定值为0xEE
			uint8_t motorID   :4;    // 4电机编号
			uint8_t work_mode :3;    // 3工作模式
			uint8_t save      :1;    // 1保留

			int16_t t         :16;  
			int16_t w         :16;  
			int32_t pos       :32;

			uint16_t kp        :16; 
			uint16_t kw        :16;			

      uint16_t CRC16    :16; // 2bytes
    } MCU2MoterMessage;

#pragma pack()

#pragma pack(1)
    /*******************************************************************
     * @brief  电机向MCU发送信息数据体结构体
     ******************************************************************/
    typedef struct
    { // 以 4个字节一组排列 ，不然编译器会凑整
        // 定义 数据
        uint8_t mode;    // 当前关节模式
        uint8_t readBit; // 电机控制参数修改     是否成功位
        int8_t temp;     // 电机当前平均温度
        uint8_t MError;  // 电机错误 标识

        uint8_t read; // 读取的当前 电机 的控制数据
        int16_t t;    // 当前实际电机输出力矩       7 + 8 描述

        int16_t w; // 当前实际电机速度（高速）   8 + 7 描述
        float lW;  // 当前实际电机速度（低速）

        int16_t w2; // 当前实际关节速度（高速）   8 + 7 描述
        float lW2;  // 当前实际关节速度（低速）

        int16_t acc;    // 电机转子加速度       15+0 描述  惯量较小
        int16_t outAcc; // 输出轴加速度         12+3 描述  惯量较大

        int32_t pos;  // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
        int32_t pos2; // 关节编码器位置(输出编码器)

        int16_t gyro[3]; // 电机驱动板6轴传感器数据
        int16_t accBd[3];

        // 力传感器的数据
        int16_t FGyro[3]; //
        int16_t FAcc[3];
        int16_t Fmag[3];
        uint8_t FTemp; // 8位表示的温度  7位（-28~100度）  1位0.5度分辨率

        int16_t force16; // 力传感器高16位数据
        int8_t force8;   // 力传感器低8位数据

        uint8_t FError; //  足端传感器错误标识

        int8_t res[1]; // 通讯 保留字节

    } ServoComd; // 加上数据包的包头 和CRC 78字节（4+70+4）

    /*******************************************************************
     * @brief  电机向MCU发送信息报文结构体:数据包头+数据体+CRC校验
     ******************************************************************/
//    typedef struct
//    {
//        // 定义 电机控制命令数据包
//        HEAD_mode_set head;
//        ServoComd Mdata;

//        uint32_t CRC32data;

//    } ServoComdData; // 发送数据

    MCU2MoterMessage *convertSetVal2Msg(MCU2MoterMessage *cmdmsg, MotorSetValTypeDef setVal);

#pragma pack()

#ifdef __cplusplus
}
#endif /* extern "C */
		
inline uint32_t crc32_core(uint32_t* ptr, uint32_t len){
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1U << 31; //U保证移位不会超出范围，否则1会被编译器自动默认设定int
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}


typedef struct{
float rad_s;
float angle;
float torque;
}mi_leg_motor;

/// @brief 电机反馈结构体
typedef struct
{
    uint16_t rxlen;                 // 从串口接收到的字节长度
    unsigned char motor_id;         // 电机ID
    unsigned char mode;             // 0:空闲 1:FOC控制 2:电机标定
    int Temp;                       // 温度
    int MError;           			// 错误码
    float T;                        // 当前实际电机输出力矩(电机本身的力矩)(Nm)
    float W;                        // 当前实际电机速度(电机本身的速度)(rad/s)
    float Pos;                      // 当前电机位置(rad)
    int correct;                    // 接收数据是否完整(1完整，0不完整)
    int footForce;                  // 足端力传感器原始数值

    RecvData_t motor_recv_data;    //电机接收数据结构体


} 	Unitee_MotorData_t;

#define R_of_ROUND 12

typedef struct{
u8 ID;
float pos;
float vel    ;
float current;
int16_t recieve:8;
}old_mi;

void leida_use(void);
MCU2MoterMessage *convertSetVal2Msg(MCU2MoterMessage *cmdmsg, const MotorSetValTypeDef setVal);
void Motor_Set_Current2_0_4(signed short int i1, signed short int i2, signed short int i3, signed short int i4); //发送
void Motor_Set_Current1_0_4(signed short int i1, signed short int i2, signed short int i3, signed short int i4); //发送
char Solution_3508(volatile dji_motor *v);
void Set_gimbal_6020(int16_t pitch,int16_t yaw);
void PHC_GIMBAL(int16_t pitch,int16_t yaw);
void extract_data(Unitee_MotorData_t *motor_r);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
void Send_to_chassic(u8 u1,u8 u2,u8 u3,u8 u4,pack_send_gimbal*abc,pack_send_mouse *pdf,u8 u5,u8 u6);
void Send_to_LMI(u8 ID,u8 CAN,float Tor);
#endif
