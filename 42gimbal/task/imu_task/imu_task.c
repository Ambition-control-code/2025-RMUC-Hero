#include "imu_task.h"
#include "MadgwickAHRS.h"
#include "string.h"
#include "QuaternionEKF.h"
#include "kalman_filter.h"
#include "bsp_dwt.h"
#include "ulitility_tools.h"
#include "BMI088driver.h"
#include "BMI088reg.h"

extern bucket_imu imu;

void imu_calculate_madgwick(void)
{
MadgwickAHRSupdateIMU(imu.gyro_buff[0],imu.gyro_buff[1],imu.gyro_buff[2],imu.acc[0],imu.acc[1],imu.acc[2]);
}

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;



// axis=sqrt(a2+b2+c2)/|a^2+b^2+c^2|;

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float* init_q4) {
  float acc_init[3] = {0};
  float gravity_norm[3] = {0, 0, 1};  // 导航系重力加速度矢量,归一化后为(0,0,1)
  float axis_rot[3] = {0};  // 旋转轴
  // 读取100次加速度计数据,取平均值作为初始值
  for (uint8_t i = 0; i < 100; ++i) {
	 BMI088_read(imu.gyro,imu.acc,&(imu.temp));
    acc_init[0] += imu.acc[0];
    acc_init[1] += imu.acc[1];
    acc_init[2] += imu.acc[2];
    DWT_Delay(0.001);
  }
  for (uint8_t i = 0; i < 3; ++i)
    acc_init[i] /= 100;
	float angle;
  Norm3d(acc_init);
//  // 计算原始加速度矢量和导航系重力加速度矢量的夹角
   angle = 0;//acosf(Dot3d(acc_init, gravity_norm));
   Cross3d(acc_init, gravity_norm, axis_rot);
   Norm3d(axis_rot);
  init_q4[0] = cosf(angle / 2.0f);
  for (uint8_t i = 0; i < 2; ++i)
    init_q4[i + 1] =
        axis_rot[i] * sinf(angle / 2.0f);  // 轴角公式,第三轴为0(没有z轴分量)
}


void INS_Init(void) {
  float init_quaternion[4] = {0};
  InitQuaternion(init_quaternion);
  IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f,0); //ekf初始化
  // imu heat init
//  INS.AccelLPF = 0.0085f;
//  INS.DGyroLPF = 0.008f;
}
/**
?* @brief ? ? ? ? ?Transform 3dvector from BodyFrame to EarthFrame
?* @param[1] ? ? ? vector in BodyFrame
?* @param[2] ? ? ? vector in EarthFrame
?* @param[3] ? ? ? quaternion
?*/
void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q) {
 vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
(q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
(q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
(0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
(q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
(q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
(0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
?* @brief ? ? ? ? ?Transform 3dvector from EarthFrame to BodyFrame
?* @param[1] ? ? ? vector in EarthFrame
?* @param[2] ? ? ? vector in BodyFrame
?* @param[3] ? ? ? quaternion
?*/
void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q) {
 vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
(q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
(q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

 vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
(0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
(q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

 vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
(q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
(0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}


void imu_task(void) {
	static int i;
	if(!i)
	{
	INS_Init();
	i++;
	}
	
	imu.AccelLPF=0.0085f;
  static uint32_t count = 0;
  const float gravity[3] = {0, 0, 9.74884605f};
  dt = DWT_GetDeltaT(&INS_DWT_Count);

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(imu.gyro_buff[0],imu.gyro_buff[1],imu.gyro_buff[2],
                             imu.acc_fliter[0],imu.acc_fliter[1],imu.acc_fliter[2]);

    memcpy(imu.q, QEKF_INS.q, sizeof(QEKF_INS.q));
	
	  imu.yaw[2]=QEKF_INS.YawTotalAngle;

//    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, imu.xn, imu.q);
    BodyFrameToEarthFrame(yb, imu.yn, imu.q);
    BodyFrameToEarthFrame(zb, imu.zn, imu.q);

//    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, imu.q);
    for (uint8_t i = 0; i < 3; i++)  // 同样过一个低通滤波
    {
      imu.MotionAccel_b[i] =
          (imu.acc_fliter[i] - gravity_b[i]) * dt / (imu.AccelLPF + dt) +
          imu.MotionAccel_b[i] * imu.AccelLPF / (imu.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(imu.MotionAccel_b, imu.MotionAccel_n,
                          imu.q);  // 转换回导航系n

//    // 获取最终数据
//    INS.Yaw = QEKF_INS.Yaw;
//    INS.Pitch = QEKF_INS.Pitch;
//    INS.Roll = QEKF_INS.Roll;
//    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
  

}


