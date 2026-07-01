#ifndef PID_IS_H
#define PID_IS_H
#include "main.h"

#define SAMPLE_PERIOD 0.002
#define V_EORROR_INTEGRAL_MAX 2000
#define P_EORROR_INTEGRAL_MAX 2000

typedef enum {
    EXPONENT,
    POWER,
    TFSMC,
    VELSMC,
    EISMC
} Rmode;

typedef struct {

    float tar_now;//当前目标值
    float tar_last;//上一次目标值
    float tar_differential;//目标值一阶导
    float tar_differential_last;//上一次目标值一阶导
    float tar_differential_second;//目标值二阶导

    float pos_get;//当前位置
    float vol_get;//当前速度

    float p_error;//位置误差
    float v_error;//速度误差（位置误差一阶导）

    float p_error_integral;//位置误差积分
    float v_error_integral;//速度误差积分
//    float v_error_integral_max; //积分限幅

    float pos_error_eps;   //误差精度
    float vol_error_eps;   //误差精度
    float error_last;
}RError;

typedef struct {
    float J;
    float K;
    float c;

    float c1;   //EIsmc参数
    float c2;   //EIsmc参数

    float p;    //tfsmc参数，正奇数 p>q
    float q;    //tfsmc参数，正奇数
    float beta; //tfsmc参数，正数
    float epsilon; //ε噪声上限
}SlidingParam;

typedef struct {
    float u; //控制器输出
    float s; //滑模面计算储存

    SlidingParam param;
    SlidingParam param_last;

    RError error;
    float u_max; //输出限幅
    Rmode flag; //符号和饱和切换，未用到
    float limit; //饱和函数上下限
}Sliding;

void 	SMC_init(SMC_control *SMC,float C,float K,float ref,float error_eps,float u_max,float J,float epsilon);
float SMC_Tick(SMC_control *SMC,float ref,float angle_now,float angle_vel);//anlge为当前位置(°),ang_vel为角速度(°/s)

void TD_Init(TD_t *td, float r, float h0);
float TD_Calculate(TD_t *td, float input);

float ABS_ABS(float a);
void  PID_IS_Init(PID* Pid,float kp,float ki,float kd,float pmax,float imax,float dmax,float outmax,float sum_error_max,float interget_seperate);
float PID_Incremental(PID* Pid,float set ,float real);
float PID_Position(PID *Pid,float real,float set);
float PID_fhan(float x1,float x2,float r,float h);
void feedforward_init(ff_type_def *ff, fp32 k1, fp32 k2, fp32 omax);
fp32 feedforward_calc(ff_type_def *ff, fp32 input);
#endif
