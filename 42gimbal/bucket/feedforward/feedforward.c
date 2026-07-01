/**
  ***************************************************************************************************
  * @file       feedforward.c/h
  * @brief      二阶差分前馈环节实现函数，包括初始化函数、计算函数和清零函数
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024/9/11     	圣灵亮欣        1. 完成
  *  V1.1.0     2024/9/16     	圣灵亮欣        1. 增加参数：输出上限
  *  V1.1.1     2024/9/20     	圣灵亮欣        1. 修复了结构体初始化时错将output_max赋值为0的问题
  *  V1.1.2     2024/10/9     	圣灵亮欣        1. 调整了前馈输出限制逻辑
  *
  @verbatim
  ===================================================================================================

  ===================================================================================================
  @endverbatim
  ***************************************************************************************************
  */
#include "feedforward.h"

/**
  * @brief          前馈结构体初始化
	* @param[out]     ff: feedforward结构数据指针
  * @param[in]      k1: 一阶差分环节系数
  * @param[in]      k2：二阶差分环节系数
  * @param[in]      omax：输出上限（绝对值）
  * @retval         none
  */
void feedforward_init(ff_type_def *ff, fp32 k1, fp32 k2, fp32 omax)
{
	if(ff == NULL)
	{
		return;
	}
	ff -> k_ff_1 = k1;
	ff -> k_ff_2 = k2;
	ff -> input_last = 0;
	ff -> output = 0;
	ff -> output_max = omax;
	ff -> dif_1 = ff -> dif_2 = 0.0f;
	ff -> dbuf[0] = ff -> dbuf[1] = 0.0f;
}

/**
  * @brief          前馈数据计算
	* @param[out]     output：前馈环节输出
  * @param[in]      ff: feedforward结构数据指针
  * @param[in]      input：前馈环节输入
  * @retval         前馈输出
  */
fp32 feedforward_calc(ff_type_def *ff, fp32 input)
{
	if(ff == NULL)
	{
		return 0.0f;
	}
	
	ff -> dif_1 = input - ff -> input_last;
	ff -> dbuf[0] = ff -> dif_1;
	ff -> dif_2 = ff -> dbuf[0] - ff -> dbuf[1];
	ff -> output = ff -> dif_1 * ff -> k_ff_1 + ff -> dif_2 * ff -> k_ff_2;
	
	ff -> input_last = input;
	ff -> dbuf[1] = ff -> dbuf[0];
	if (ff -> output > ff -> output_max)
	{                      
			ff -> output = ff -> output_max;
	}
	else if (ff -> output < -(ff -> output_max))
	{
			ff -> output = -(ff -> output_max);
	}
	return ff -> output;
}

/**
  * @brief          前馈状态清零
	* @param[out]     ff: feedforward结构数据指针
  * @retval         none
  */
void feedforward_clear(ff_type_def *ff)
{
	if(ff == NULL)
	{
		return;
	}
	ff -> input_last = 0;
	ff -> output = 0;
	ff -> dif_1 = ff -> dif_2 = 0.0f;
	ff -> dbuf[0] = ff -> dbuf[1] = 0.0f;
}
	