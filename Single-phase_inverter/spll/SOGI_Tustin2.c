/*
 * SOGI_Tustin2.c
 *
 *  Created on: 2025年11月11日
 *      Author: LW
 */

// --------------------------------------
// 头文件
// --------------------------------------
#include "F28x_Project.h"

// --------------------------------------
// 变量定义
// --------------------------------------
SOGI_StateSpace sogi_statespace;


// --------------------------------------
// 变量声明
// --------------------------------------



// --------------------------------------
// 函数定义
// --------------------------------------


// =============================================================================
/*******************************************************************************
 函数名称：   void SOGI_StateSpace_reset(SOGI_StateSpace *sogi_statespace)
 功能描述：   重置基于状态方程法的SOGI所有状态变量
 输入参数：   sogi_statespace : 状态方程SOGI结构体指针
 输出参数：   无
 返 回 值：  无
 其它说明：   a. 用于状态方程法离散化的SOGI复位
           b. 清除所有历史状态变量
           c. 确保算法从零状态开始
 修改日期        版本号            修改人            修改内容
 -----------------------------------------------------------------------------
 2025/11/11    V1.0           LW              创建
********************************************************************************/
void SOGI_StateSpace_reset(SOGI_StateSpace *sogi_statespace)
{
    // 电压状态变量清零
    sogi_statespace->osg_u[0] = (float32_t)(0.0);
    sogi_statespace->osg_u[1] = (float32_t)(0.0);
    sogi_statespace->osg_qu[0] = (float32_t)(0.0);
    sogi_statespace->osg_qu[1] = (float32_t)(0.0);

    // 电流状态变量清零
    sogi_statespace->osg_i[0] = (float32_t)(0.0);
    sogi_statespace->osg_i[1] = (float32_t)(0.0);
    sogi_statespace->osg_qi[0] = (float32_t)(0.0);
    sogi_statespace->osg_qi[1] = (float32_t)(0.0);

    // 输入信号历史清零
    sogi_statespace->u[0] = (float32_t)(0.0);
    sogi_statespace->u[1] = (float32_t)(0.0);
    sogi_statespace->i[0] = (float32_t)(0.0);
    sogi_statespace->i[1] = (float32_t)(0.0);
}

// =============================================================================
/*******************************************************************************
 函数名称：   void SOGI_StateSpace_coeff_calc(SOGI_StateSpace *sogi_statespace)
 功能描述：   基于状态空间方程和梯形积分法（双线性变换）计算SOGI离散化系数
             用于生成与输入信号同相和正交的分量
 输入参数：   sogi_statespace : 指向SOGI状态空间结构体的指针
 输出参数：   无
 返 回 值：  无
 其它说明：
             a. 采用状态空间方程和梯形积分法（双线性变换）进行离散化
             b. 连续时间状态方程：
                   dx1/dt = ω[k(u - x1) - x2]  --> 同相分量
                   dx2/dt = ωx1                 --> 正交分量
             c. 离散化后得到递归计算形式：
                   osg_u[n] = a11*osg_u[n-1] + a12*osg_qu[n-1] + b1*(u[n] + u[n-1])
                   osg_qu[n] = osg_qu[n-1] + a21*(osg_u[n] + osg_u[n-1])
 设计原理：
             1. 通过双线性变换将连续时间系统映射到离散时间系统
             2. 计算状态空间离散化系数，确保数值稳定性和计算效率
 修改日期        版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/11/11    V1.0           LW              创建
********************************************************************************/
void SOGI_StateSpace_coeff_calc(SOGI_StateSpace *sogi_statespace)
{

    // 计算基本系数
    sogi_statespace->coeff.k = (float32_t)(1.0);               // 阻尼系数固定为1.0
    sogi_statespace->coeff.wn = PI2 * sogi_statespace->fn;     // 计算中心角频率：ωn = 2π * fn

    // 计算中间变量，提高计算效率
    sogi_statespace->coeff.kTw = sogi_statespace->coeff.k * sogi_statespace->delta_t * sogi_statespace->coeff.wn;                    // k * T * ωn
    sogi_statespace->coeff.T2w2 = (sogi_statespace->delta_t * sogi_statespace->coeff.wn) *
                                   (sogi_statespace->delta_t * sogi_statespace->coeff.wn);          // (T * ωn)²

    // 计算分母系数：D = 1 + (k*T*ωn)/2 + (T²*ωn²)/4
    sogi_statespace->coeff.D = (float32_t)(1.0) +
                               sogi_statespace->coeff.kTw / (float32_t)(2.0) +
                               sogi_statespace->coeff.T2w2 / (float32_t)(4.0);

    // =========================================================================
    // 计算状态更新系数（离散化后的递归系数）
    // =========================================================================

    // 同相分量系数：
    // a11 = [1 - (k*T*ωn)/2 - (T²*ωn²)/4] / D
    sogi_statespace->coeff.a11 = ((float32_t)(1.0) -
                       sogi_statespace->coeff.kTw / (float32_t)(2.0) -
                       sogi_statespace->coeff.T2w2 / (float32_t)(4.0)) / sogi_statespace->coeff.D;

    // 正交分量耦合项系数：
    // a12 = (-T * ωn) / D
    sogi_statespace->coeff.a12 = ((-sogi_statespace->delta_t) * sogi_statespace->coeff.wn) / sogi_statespace->coeff.D;

    // 输入信号系数：
    // b1 = (k*T*ωn/2) / D
    sogi_statespace->coeff.b1 = (sogi_statespace->coeff.kTw / (float32_t)(2.0)) / sogi_statespace->coeff.D;

    // 正交分量系数：
    // a21 = (T * ωn) / 2
    sogi_statespace->coeff.a21 = (sogi_statespace->delta_t * sogi_statespace->coeff.wn) / (float32_t)(2.0);
}


// =============================================================================
/*******************************************************************************
 函数名称：   void SOGI_StateSpace_config(SOGI_StateSpace *sogi_statespace,
                                        float32_t acFreq,
                                        float32_t isrFrequency)
 功能描述：   配置基于状态方程法的SOGI系统参数
 输入参数：   sogi_statespace:   状态方程SOGI结构体指针
           acFreq: 电网标称频率（Hz）
           isrFrequency: 中断服务例程频率（Hz）
 输出参数：   无
 返 回 值：  无
 其它说明：   a. 专门用于状态方程法的SOGI参数配置
           b. 状态方程法具有明确的物理意义，便于理解系统动态特性
 状态方程法优势：
          - 物理意义明确，状态变量直接对应系统物理量
          - 数值稳定性好
          - 便于状态观测器和控制器设计
 修改日期          版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/11/4      V1.0           LW              创建
********************************************************************************/
void SOGI_StateSpace_config(SOGI_StateSpace *sogi_statespace,
                           float32_t acFreq,
                           float32_t isrFrequency)
{
    // 设置电网标称频率（Hz）
    sogi_statespace->fn = acFreq;

    // 计算采样周期 Ts = 1 / Fs
    sogi_statespace->delta_t = ((float32_t)(1.0)) / isrFrequency;

    // 基于状态方程法计算SOGI离散系数
    SOGI_StateSpace_coeff_calc(sogi_statespace);
}


// =============================================================================
/*******************************************************************************
 函数名称：   void SOGI_StateSpace_alpha_beta(SOGI_StateSpace *sogi_statespace,
                                           float32_t voltage_Value,
                                           float32_t current_value)
 功能描述：   实现基于状态方程法的二阶广义积分器(SOGI)算法
           生成电压和电流的正交信号（α-β分量）
 输入参数：   sogi_statespace    : SOGI状态空间结构体指针
           voltage_Value : 交流电压瞬时采样值
           current_value: 交流电流瞬时采样值
 输出参数：   无
 返 回 值：  无
 其它说明：   a. 采用状态方程法和梯形积分法实现离散SOGI算法
           b. 同时处理电压和电流信号，生成各自的正交分量
 算法原理:
     SOGI通过状态方程产生与输入信号同相的α分量和滞后90°的β分量
     形成虚拟的α-β两相坐标系，便于后续的锁相环计算

     离散化方法: 状态方程法 + 梯形积分法
     结构特点: 状态变量直接对应物理量，结构清晰
 修改日期         版本号             修改人            修改内容
 -----------------------------------------------------------------------------
 2025/11/04     V1.0           LW              创建
********************************************************************************/
void SOGI_StateSpace_alpha_beta(SOGI_StateSpace *sogi_statespace,
                               float32_t voltage_Value,
                               float32_t current_value)
{
    // 更新输入电压历史
    sogi_statespace->u[0] = voltage_Value;

    //----------------------------------------------------------------------
    // 电压正交信号发生器（SOGI核心）
    //----------------------------------------------------------------------

    // 计算电压同相分量（α分量）状态：osg_u[n]
    // 状态方程：osg_u[n] = a11*osg_u[n-1] + a12*osg_qu[n-1] + b1*(u[n] + u[n-1])
    sogi_statespace->osg_u[0] = (sogi_statespace->coeff.a11 * sogi_statespace->osg_u[1]) +
                    (sogi_statespace->coeff.a12 * sogi_statespace->osg_qu[1]) +
                    (sogi_statespace->coeff.b1 * (sogi_statespace->u[0] + sogi_statespace->u[1]));

    // 计算电压正交分量（β分量）状态：osg_qu[n]
    // 状态方程：osg_qu[n] = osg_qu[n-1] + a21*(osg_u[n] + osg_u[n-1])
    sogi_statespace->osg_qu[0] = sogi_statespace->osg_qu[1] +
                   (sogi_statespace->coeff.a21 * (sogi_statespace->osg_u[0] + sogi_statespace->osg_u[1]));

    // 更新电压历史状态
    sogi_statespace->osg_u[1] = sogi_statespace->osg_u[0];    // 将当前时刻值osg_u[0]移至上一时刻位置osg_u[1]
    sogi_statespace->osg_qu[1] = sogi_statespace->osg_qu[0];  // 将当前时刻值osg_qu[0]移至上一时刻位置osg_qu[1]

    // 更新电压输入
    sogi_statespace->u[1] = sogi_statespace->u[0];        // 将当前时刻输入值u[0]移至上一时刻位置u[1]

    //----------------------------------------------------------------------
    // 电流正交信号发生器（SOGI核心）
    //----------------------------------------------------------------------

    // 更新输入电流历史
    sogi_statespace->i[0] = current_value;

    // 计算电流同相分量（α分量）状态:osg_i[n]
    sogi_statespace->osg_i[0] = (sogi_statespace->coeff.a11 * sogi_statespace->osg_i[1]) +
                    (sogi_statespace->coeff.a12 * sogi_statespace->osg_qi[1]) +
                    (sogi_statespace->coeff.b1 * (sogi_statespace->i[0] + sogi_statespace->i[1]));

    // 计算电流正交分量（β分量）状态:osg_qi[n]
    sogi_statespace->osg_qi[0] = sogi_statespace->osg_qi[1] +
                   (sogi_statespace->coeff.a21 * (sogi_statespace->osg_i[0] + sogi_statespace->osg_i[1]));

    // 更新电流历史状态
    sogi_statespace->osg_i[1] = sogi_statespace->osg_i[0];    // 将当前时刻值osg_i[0]移至上一时刻位置osg_i[1]
    sogi_statespace->osg_qi[1] = sogi_statespace->osg_qi[0];  // 将当前时刻值osg_qi[0]移至上一时刻位置osg_qi[1]

    // 更新电流输入
    sogi_statespace->i[1] = sogi_statespace->i[0];        // 将当前时刻输入值i[0]移至上一时刻位置i[1]

    //输出赋值
     str_alpha_beta.wValpha = sogi_statespace->osg_u[0];   // 电压α分量
     str_alpha_beta.wVbeta  = sogi_statespace->osg_qu[0];   // 电压β分量
     str_alpha_beta.wIalpha = sogi_statespace->osg_i[0];   // 电流α分量
     str_alpha_beta.wIbeta  = sogi_statespace->osg_qi[0];   // 电流β分量
}



// ---------------------------------
// End of File
// ---------------------------------


