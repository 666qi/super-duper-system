/*
 * SOGI_Tustin1.h
 *
 *  Created on: 2025年11月4日
 *      Author: LW
 */

#ifndef SOGI_H_
#define SOGI_H_



// --------------------------------------
// 头文件
// --------------------------------------
#include "F28x_Project.h"

// --------------------------------------
// 宏定义
// --------------------------------------

// --------------------------------------
// 自定义变量
// --------------------------------------

//定义SOGI正交信号发生器系数结构体
typedef struct{
    float32_t osg_k;    // SOGI阻尼系数k（固定为1.0）
    float32_t osg_x;    // 中间计算变量：2*k*ωn*Ts
    float32_t osg_y;    // 中间计算变量：(ωn*Ts)^2
    float32_t osg_b0;   // 同相分量分子系数b0
    float32_t osg_b2;   // 同相分量分子系数b2
    float32_t osg_a1;   // 分母系数a1
    float32_t osg_a2;   // 分母系数a2
    float32_t osg_qb0;  // 正交分量分子系数qb0
    float32_t osg_qb1;  // 正交分量分子系数qb1
    float32_t osg_qb2;  // 正交分量分子系数qb2
} OSG_COEFF;

//定义SOGI正交信号发生器变量结构体
typedef struct{

    float32_t   u[3];       // 交流电压输入数据缓冲区（当前和前两个采样值）
    float32_t   osg_u[3];   // 正交信号发生器同相分量数据缓冲区
    float32_t   osg_qu[3];  // 正交信号发生器正交分量数据缓冲区

    float32_t   i[3];       // 交流电流输入数据缓冲区（当前和前两个采样值）
    float32_t   osg_i[3];   // 电流正交信号发生器同相分量数据缓冲区
    float32_t   osg_qi[3];  // 电流正交信号发生器正交分量数据缓冲区

    float32_t   fn;         // 中频率(Hz)
    float32_t   delta_t;    // 采样周期

    OSG_COEFF osg_coeff; // 正交信号发生器系数

} SOGI_Tustin;

// --------------------------------------
// 变量声明
// --------------------------------------
extern SOGI_Tustin sogi_tustin;  // 声明为外部变量

// --------------------------------------
// 函数声明
// --------------------------------------

extern void SOGI_Tustin_reset(SOGI_Tustin *sogi_tustin);
extern void SOGI_Tustin_coeff_calc(SOGI_Tustin *sogi_tustin);
extern void SOGI_Tustin_config(SOGI_Tustin *sogi_tustin,float32_t acFreq, float32_t isrFrequency);
extern void SOGI_Tustin_alpha_beta(SOGI_Tustin *sogi_tustin, float32_t voltage_Value, float32_t current_value);


#endif /* SOGI_H_ */
