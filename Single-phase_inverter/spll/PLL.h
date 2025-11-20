/*
 * PLL.h
 *
 *  Created on: 2025年9月2日
 *      Author: LW
 */

#ifndef SPLL_PLL_H_
#define SPLL_PLL_H_

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

// 保存 d轴 q轴 的数据
typedef struct
{
    float32_t wVd; // d轴电压分量（单位：V）
    float32_t wId; // d轴电流分量（单位：A）

    float32_t wVq;  // q轴电压分量（单位：V）
    float32_t wIq;  // q轴电流分量（单位：A）

}str_d_qDef;


//SOGI_PLL变量结构体定义
typedef struct
{
    // PLL_PID参数
    float32_t wkp;            // PLL比例系数
    float32_t wki;            // PLL积分系数

    // PLL状态变量
    float32_t err;           // 相位误差
    float32_t err_last;      // 上一周期误差
    float32_t pi_out;        // PI控制器输出
    float32_t w_internal;    // 内部角频率变量

}str_SOGI_PLLDef;


// --------------------------------------
// 全局变量声明
// --------------------------------------

//d-q轴 变量
extern str_d_qDef          str_d_q;

//SOGI_PLL 变量

extern str_SOGI_PLLDef    str_SOGI_PLL;


// --------------------------------------
// 函数声明
// --------------------------------------
void Init_SOGI_PLL(float32_t kp, float32_t ki,float32_t freq, float32_t init_err,
                   float32_t init_last_err);
void Delay_PLL(void);
void SOGI_PLL(void);


#endif /* PLL_H_ */
