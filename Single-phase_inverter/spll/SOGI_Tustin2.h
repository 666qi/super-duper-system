/*
 * SOGI_Tustin2.h
 *
 *  Created on: 2025年11月11日
 *      Author: LW
 */

#ifndef SPLL_SOGI_TUSTIN2_H_
#define SPLL_SOGI_TUSTIN2_H_

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
//SOGI状态方程系数结构体
typedef struct {

    float32_t k;        // 阻尼系数
    float32_t wn;       // 角频率ωn

    // 中间计算变量
    float32_t kTw;      // 中间变量：k*T*ω
    float32_t T2w2;     // 中间变量：(T*ω)^2
    float32_t D;        // 分母系数

    // 状态更新系数
    float32_t a11;      // x1[n]计算中x1[n-1]的系数
    float32_t a12;      // x1[n]计算中x2[n-1]的系数
    float32_t b1;       // x1[n]计算中输入项的系数
    float32_t a21;      // x2[n]计算中x1项的系数

} SOGI_StateSpace_Coeff;


//SOGI状态方程系数结构体
typedef struct {

    // 电压状态变量
    float32_t osg_u[2];      // 电压同相分量状态 [当前值, 上一时刻值]
    float32_t osg_qu[2];      // 电压正交分量状态 [当前值, 上一时刻值]

    // 电流状态变量
    float32_t osg_i[2];      // 电流同相分量状态 [当前值, 上一时刻值]
    float32_t osg_qi[2];      // 电流正交分量状态 [当前值, 上一时刻值]

    // 输入信号历史
    float32_t u[2];         // 电压输入历史 [当前值, 上一时刻值]
    float32_t i[2];         // 电流输入历史 [当前值, 上一时刻值]

    // 系统参数
    float32_t fn;           // 中心频率(Hz)
    float32_t delta_t;      // 采样时间(s)

    // 系数结构体
    SOGI_StateSpace_Coeff coeff;  //状态方程系数

} SOGI_StateSpace;

// --------------------------------------
// 全局变量声明
// --------------------------------------
extern SOGI_StateSpace sogi_statespace;  // 声明为外部变量


// --------------------------------------
// 函数声明
// --------------------------------------

void SOGI_StateSpace_reset(SOGI_StateSpace *sogi_statespace);
void SOGI_StateSpace_coeff_calc(SOGI_StateSpace *sogi_statespace);
void SOGI_StateSpace_config(SOGI_StateSpace *sogi_statespace,
                           float32_t acFreq,
                           float32_t isrFrequency);
void SOGI_StateSpace_alpha_beta(SOGI_StateSpace *sogi_statespace,
                               float32_t voltage_Value,
                               float32_t current_value);

#endif /* SPLL_SOGI_TUSTIN2_H_ */
