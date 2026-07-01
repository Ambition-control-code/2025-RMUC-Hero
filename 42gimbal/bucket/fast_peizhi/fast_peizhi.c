#include "ulitility_tools.h"
#include "stdlib.h"
#include "string.h"

//void judege_station(int16_t a,int16_t b,Gimbal_hero *HERO_AMB)
//{
// if((!a)&(!b))
// {
//	HERO_AMB->now_gimbal_station=0;
//	HERO_AMB->chassic_mode=0;
// }
// else if(a==2)
// {
//	HERO_AMB->now_gimbal_station=1;	
// }
// else if(a==1)
// {
//	HERO_AMB->now_gimbal_station=2;
// }
//}

float gimbal_pitch_replay(float now_pitch,float *a)
{
	float output;
		if(now_pitch>-27)
		{                                                 
		 if(now_pitch<0)
		 {
  output=*(a+1)+ABS(now_pitch)*(*(a+2)-*(a+1))/27.0f;
		 }
		 else
		 {
output=*a+now_pitch*(*(a+1)-*a)/15.0f;
		 }
		}
		else{output=*(a+2);}
		return output;
}

/**
  * @brief          最小二乘法初始化
  * @param[in]      最小二乘法结构体
  * @param[in]      样本数
  * @retval         返回空
  */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)malloc(sizeof(float) * order);
    OLS->y = (float *)malloc(sizeof(float) * order);
    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
  * @brief          最小二乘法平滑信号
  * @param[in]      最小二乘法结构体
  * @param[in]      信号新样本距上一个样本时间间隔
  * @param[in]      信号值
  * @retval         返回平滑输出
  */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

		OLS->result=OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
		
    return OLS->result;
}


//#define delta_T     0.001f  // 采样周期1ms 即频率1KHZ
//#define PI          3.1415926f

//float I_ex, I_ey, I_ez;  // 误差积分
//quater_param_t Q_info = {1, 0, 0, 0};  // 四元数初始化
//euler_param_t eulerAngle;              // 欧拉角
//icm_param_t icm_data;                  // ICM20602采集的六轴数值
//float icm_kp= 0.17;    // 加速度计的收敛速率比例增益 
//float icm_ki= 0.004;   // 陀螺仪收敛速率的积分增益
///**
// * @brief 用互补滤波算法解算陀螺仪姿态(即利用加速度计修正陀螺仪的积分误差)
// * 加速度计对振动之类的噪声比较敏感，长期数据计算出的姿态可信；陀螺仪对振动噪声不敏感，短期数据可信，但长期使用积分误差严重(内部积分算法放大静态误差)。
// * 因此使用姿态互补滤波，短期相信陀螺仪，长期相信加速度计。
// * @tips: n - 导航坐标系； b - 载体坐标系
// */
//void icmAHRSupdate(icm_param_t* icm)            
//{
//    float halfT = 0.5 * delta_T;    // 采样周期一半
//    float vx, vy, vz;               // 当前姿态计算得来的重力在三轴上的分量
//    float ex, ey, ez;               // 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差
//    
//    float q0 = Q_info.q0;  //四元数
//    float q1 = Q_info.q1;
//    float q2 = Q_info.q2;
//    float q3 = Q_info.q3;
//    
//    float q0q0 = q0 * q0;  //先相乘，方便后续计算
//    float q0q1 = q0 * q1;
//    float q0q2 = q0 * q2;
//    float q0q3 = q0 * q3;
//    float q1q1 = q1 * q1;
//    float q1q2 = q1 * q2;
//    float q1q3 = q1 * q3;
//    float q2q2 = q2 * q2;
//    float q2q3 = q2 * q3;
//    float q3q3 = q3 * q3;

//    // 正常静止状态为-g 反作用力。
//    if(icm->acc_x * icm->acc_y * icm->acc_z == 0) // 加计处于自由落体状态时(此时g = 0)不进行姿态解算，因为会产生分母无穷大的情况
//        return;

//    // 对加速度数据进行归一化 得到单位加速度 (a^b -> 载体坐标系下的加速度)
//    float norm = myRsqrt(icm->acc_x * icm->acc_x + icm->acc_y * icm->acc_y + icm->acc_z * icm->acc_z); 
//    icm->acc_x = icm->acc_x * norm;
//    icm->acc_y = icm->acc_y * norm;
//    icm->acc_z = icm->acc_z * norm;

//    // 载体坐标系下重力在三个轴上的分量
//    vx = 2 * (q1q3 - q0q2);
//    vy = 2 * (q0q1 + q2q3);
//    vz = q0q0 - q1q1 - q2q2 + q3q3;

//    // g^b 与 a^b 做向量叉乘，得到陀螺仪的校正补偿向量e的系数
//    ex = icm->acc_y * vz - icm->acc_z * vy;
//    ey = icm->acc_z * vx - icm->acc_x * vz;
//    ez = icm->acc_x * vy - icm->acc_y * vx;

//    // 误差累加
//    I_ex += halfT * ex;  
//    I_ey += halfT * ey;
//    I_ez += halfT * ez;

//    // 使用PI控制器消除向量积误差(陀螺仪漂移误差)
//    icm->gyro_x = icm->gyro_x + icm_kp* ex + icm_ki* I_ex;
//    icm->gyro_y = icm->gyro_y + icm_kp* ey + icm_ki* I_ey;
//    icm->gyro_z = icm->gyro_z + icm_kp* ez + icm_ki* I_ez;

//    // 一阶龙格库塔法求解四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为b系陀螺仪角速度。
//    q0 = q0 + (-q1 * icm->gyro_x - q2 * icm->gyro_y - q3 * icm->gyro_z) * halfT;
//    q1 = q1 + (q0 * icm->gyro_x + q2 * icm->gyro_z - q3 * icm->gyro_y) * halfT;
//    q2 = q2 + (q0 * icm->gyro_y - q1 * icm->gyro_z + q3 * icm->gyro_x) * halfT;
//    q3 = q3 + (q0 * icm->gyro_z + q1 * icm->gyro_y - q2 * icm->gyro_x) * halfT;

//    // 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，下面算法类似线性代数里的正交变换
//    norm = myRsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//    Q_info.q0 = q0 * norm;
//    Q_info.q1 = q1 * norm;
//    Q_info.q2 = q2 * norm;
//    Q_info.q3 = q3 * norm;  // 用全局变量记录上一次计算的四元数值
//}

		
//void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
//	float recipNorm;
//	float halfvx, halfvy, halfvz;
//	float halfex, halfey, halfez;
//	float qa, qb, qc;

//	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

//		// Normalise accelerometer measurement
//		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
//		ax *= recipNorm;
//		ay *= recipNorm;
//		az *= recipNorm;        

//		// Estimated direction of gravity and vector perpendicular to magnetic flux
//		halfvx = q[1] * q[3] - q[0] * q[2];
//		halfvy = q[0] * q[1] + q[2] * q[3];
//		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
//	
//		// Error is sum of cross product between estimated and measured direction of gravity
//		halfex = (ay * halfvz - az * halfvy);
//		halfey = (az * halfvx - ax * halfvz);
//		halfez = (ax * halfvy - ay * halfvx);

//		// Compute and apply integral feedback if enabled
//		if(twoKi > 0.0f) {
//			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
//			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
//			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
//			gx += integralFBx;	// apply integral feedback
//			gy += integralFBy;
//			gz += integralFBz;
//		}
//		else {
//			integralFBx = 0.0f;	// prevent integral windup
//			integralFBy = 0.0f;
//			integralFBz = 0.0f;
//		}

//		// Apply proportional feedback
//		gx += twoKp * halfex;
//		gy += twoKp * halfey;
//		gz += twoKp * halfez;
//	}
//	
//	// Integrate rate of change of quaternion
//	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
//	gy *= (0.5f * (1.0f / sampleFreq));
//	gz *= (0.5f * (1.0f / sampleFreq));
//	qa = q[0];
//	qb = q[1];
//	qc = q[2];
//	q[0] += (-qb * gx - qc * gy - q[3] * gz);
//	q[1] += (qa * gx + qc * gz - q[3] * gy);
//	q[2] += (qa * gy - qb * gz + q[3] * gx);
//	q[3] += (qa * gz + qb * gy - qc * gx); 
//	
//	// Normalise quaternion
//	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
//	q[0] *= recipNorm;
//	q[1] *= recipNorm;
//	q[2] *= recipNorm;
//	q[3] *= recipNorm;
//}
