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
#ifndef __FEEDFORWARD_H_
#define __FEEDFORWARD_H_

#include "struct_typedef.h"
#include "main.h"

typedef struct
{
	fp32 k_ff_1;			//一阶差分环节系数
	fp32 k_ff_2;			//二阶差分环节系数
	
	fp32 dif_1;				//一阶差分量
	fp32 dif_2;				//二阶差分量
	
	fp32 input_last;	//前一次的输入
	fp32 output;			//输出
	fp32 output_max;	//输出上限（绝对值）
	
	fp32 dbuf[2];			//（一阶）差分项，0：最新。1：上一次
}ff_type_def;


/**
  * @brief          前馈结构体初始化
	* @param[out]     ff: feedforward结构数据指针
  * @param[in]      k1: 一阶差分环节系数
  * @param[in]      k2：二阶差分环节系数
  * @param[in]      omax：输出上限（绝对值）
  * @retval         none
  */
extern void feedforward_init(ff_type_def *ff, fp32 k1, fp32 k2, fp32 omax);

/**
  * @brief          前馈数据计算
	* @param[out]     output：前馈环节输出
  * @param[in]      ff: feedforward结构数据指针
  * @param[in]      input：前馈环节输入
  * @retval         前馈输出
  */
extern fp32 feedforward_calc(ff_type_def *ff, fp32 input);

/**
  * @brief          前馈状态清零
	* @param[out]     ff: feedforward结构数据指针
  * @retval         none
  */
extern void feedforward_clear(ff_type_def *ff);

#endif
