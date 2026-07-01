#include "speed_kalman_task.h"
#include "motor.h"
#include "VMC_calc.h"
#include "cmsis_os.h"
#include "balance_task.h"

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.1f, 0.0f, 
                           0.0f, 0.1f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                            0.0f,  100.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
														 															 
extern bucket_imu imu;		
extern mi_leg_motor MI_LEFT,MI_RIGHT;
extern balance_leg leg_ambition;
																 
extern vmc_leg_t VMC_right;			
extern vmc_leg_t VMC_left;	

float vel_acc[2]; 
uint32_t OBSERVE_TIME=2;//任务周期是3ms															 
void 	speed_kalman_task(void)
{
  static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	static float aver_v=0.0f;
		
	xvEstimateKF_Init(&vaEstimateKF);
	
  while(1)
	{  
		wr= -MI_RIGHT.rad_s-imu.gyro_buff[0]+VMC_right.d_alpha;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vrb=wr*0.0603f+VMC_right.L0*VMC_right.d_theta*arm_cos_f32(VMC_right.theta)+VMC_right.d_L0*arm_sin_f32(VMC_right.theta);//机体b系的速度
		
		wl= -MI_LEFT.rad_s+imu.gyro_buff[0]+VMC_left.d_alpha;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vlb=wl*0.0603f+VMC_left.L0*VMC_left.d_theta*arm_cos_f32(VMC_left.theta)+VMC_left.d_L0*arm_sin_f32(VMC_left.theta);//机体b系的速度

    leg_ambition.real_speed=(MI_LEFT.rad_s-MI_RIGHT.rad_s)*(-0.1)/2.0f;//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
    leg_ambition.real_location+=leg_ambition.real_speed*((float)OBSERVE_TIME/1000.0f);
	
		osDelay(OBSERVE_TIME);
	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}


