/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力



//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算


#include <math.h>
#include <stdio.h>

#include "SolveTrajectory.h"
#include "ulitility_tools.h"


struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.5f; // 飞行时间



/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    //t为给定v与angle时的飞行时间
    t = 0.01;
    if(t < 0)
    {
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
//        printf("[WRAN]: Exceeding the maximum range!\n");
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
//    printf("model %f %f\n", t, z);
    return z;
}


/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float completeAirResistanceModel(float s, float v, float angle)
{

    return 0;

}



/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
//        printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
//            i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
    int idx = 0; // 选择的装甲板
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{

    // 线性预测
    float timeDelay = 0.6;
//    st.tar_yaw = st.tar_yaw;

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = st.tar_yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


    } else {

        for (i = 0; i<4; i++) {
            float tmp_yaw = st.tar_yaw + i * PI/2.0;
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板

//计算距离最近的装甲板
	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
	int idx = 0;
	for (i = 1; i<4; i++)
	{
		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
		if (temp_dis_diff < dis_diff_min)
		{
			dis_diff_min = temp_dis_diff;
			idx = i;
		}
	}


            //计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//        for (i = 1; i<4; i++) {
//            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//            if (temp_yaw_diff < yaw_diff_min)
//            {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//        }

    }

	

    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
    *aim_y = tar_position[idx].y + st.vyw * timeDelay;
    //这里符号给错了
    float temp_pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
            *aim_z + st.z_bias, st.current_v);
    if(temp_pitch)
        *pitch = temp_pitch;
    if(*aim_x || *aim_y)
        *yaw = (float)(atan2(*aim_y, *aim_x));
}

// 从坐标轴正向看向原点，逆时针方向为正


/**
 * @brief  Calculate Bullet Model Height
 * @param  distance: distance of target armor
 * @param  bullet_k: ballistic coefficient
 * @param  bullet_speed: bullet speeed from referee
 * @param  bullet_Time： pointer to the ballistic time
 * @param  pitchangle:  target pitch angle
 * @retval Bullet Model Height in radians
 * @note   t = (e^(v*x)-1)/(k*v*cos(pitch))
 *         Height = v*sin(pitch)*t - 1/2*g*t^2
 */
float Trajectory_BulletModel(float distance,float bullet_k,float bullet_speed,float bullet_Time,float pitchangle)
{
    float height = 0.f;
    bullet_Time = (float)((exp(bullet_k * distance)- 1.f) / (bullet_k * bullet_speed * cos(pitchangle)));
    height = (float)(bullet_speed * sinf(pitchangle) * bullet_Time - 9.794f * bullet_Time * bullet_Time / 2.f);
    return height;
}

float height_calc = 0.f,pitchangle = 0.f;

float Trajectory_Picth_Update(float distance,float height)
{
    float z_temp = 0.f,dz = 0.f;
    z_temp = height;
    for(uint8_t i=0; i<200; i++)
    {
        pitchangle = (float)atan2(z_temp,distance);
        height_calc = Trajectory_BulletModel(distance,0.019f,12,0.63,pitchangle);
        dz = 0.3f*(height - height_calc);
        z_temp += dz;
        if(fabs(dz) < 0.001f)
        {
            break;
        }
    }
    return pitchangle;
}         

float k_dis_relation;
float k_bias;


float attack_bias_time=0.9f;
float last_aim;
float if_turn;
float turn_speed;

float attack_check(float middle_yaw,float speed,float now_aim,float now_yaw)
{
float attack_speed,speed_sum;
float relative_error=middle_yaw-now_aim;
turn_speed=fabs(last_aim-now_aim);
if(fabs(now_yaw-now_aim)<2.5f) {attack_speed=1,speed_sum++;}
if_turn=(fabs(last_aim-now_aim)>2) ? 1:0;



last_aim=now_aim;
return  ((attack_speed*speed_sum)>8) ? 8 :(attack_speed*speed_sum);
}

float move_fire(float error,float *dis)
{
if(st.tracking<1) return 0;
if(st.vyw>0.05) *dis=2.1; 
else if(st.vyw<-0.05) *dis=-2.1;
else *dis=0; 
if(fabsf(error)<1.7) 
return 0.5;
else
return 0;
}




