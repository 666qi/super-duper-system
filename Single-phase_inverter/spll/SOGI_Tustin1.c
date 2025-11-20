/*
 * SOGI_Tustin1.c
 *
 *  Created on: 2025年11月4日
 *      Author: LW
 */

// --------------------------------------
// 头文件
// --------------------------------------

#include "F28x_Project.h"


// --------------------------------------
// 变量定义
// --------------------------------------

SOGI_Tustin sogi_tustin;  // sogi_tustin结构体定义

// --------------------------------------
// 函数定义
// --------------------------------------

// =============================================================================
/*******************************************************************************
 函数名称：   void SOGI_Tustin_reset(SOGI_Tustin *sogi_tustin)
 功能描述：   重置Tustin方法SOGI滤波器的所有状态变量和历史缓冲区
           将滤波器恢复到初始零状态
 输入参数：   sogi_tustin : SOGI_Tustin结构体指针
 输出参数：   无
 返 回 值：  无
 其它说明：   a. 清除所有历史状态变量，确保滤波器从零状态开始工作
           b. 保持滤波器系数不变，仅重置状态变量

 重置内容：
     - 电压输入信号历史缓冲区
     - 电压同相分量（α分量）历史状态
     - 电压正交分量（β分量）历史状态
     - 电流输入信号历史缓冲区
     - 电流同相分量（α分量）历史状态
     - 电流正交分量（β分量）历史状态
 修改日期        版本号            修改人            修改内容
 -----------------------------------------------------------------------------
 2025/11/4    V1.0           LW              创建
********************************************************************************/
void SOGI_Tustin_reset(SOGI_Tustin *sogi_tustin)
{
    // 电压输入信号历史缓冲区清零（三阶缓冲区）
    sogi_tustin->u[0]=(float32_t)(0.0);
    sogi_tustin->u[1]=(float32_t)(0.0);
    sogi_tustin->u[2]=(float32_t)(0.0);

    // 电压同相分量（α分量）历史缓冲区清零
    sogi_tustin->osg_u[0]=(float32_t)(0.0);
    sogi_tustin->osg_u[1]=(float32_t)(0.0);
    sogi_tustin->osg_u[2]=(float32_t)(0.0);

    // 电压正交分量（β分量）历史缓冲区清零
    sogi_tustin->osg_qu[0]=(float32_t)(0.0);
    sogi_tustin->osg_qu[1]=(float32_t)(0.0);
    sogi_tustin->osg_qu[2]=(float32_t)(0.0);

    // 电流输入信号历史缓冲区清零（三阶缓冲区）
    sogi_tustin->i[0] = (float32_t)(0.0);
    sogi_tustin->i[1] = (float32_t)(0.0);
    sogi_tustin->i[2] = (float32_t)(0.0);

    // 电流同相分量（α分量）历史缓冲区清零
    sogi_tustin->osg_i[0] = (float32_t)(0.0);
    sogi_tustin->osg_i[1] = (float32_t)(0.0);
    sogi_tustin->osg_i[2] = (float32_t)(0.0);

    // 电流正交分量（β分量）历史缓冲区清零
    sogi_tustin->osg_qi[0] = (float32_t)(0.0);
    sogi_tustin->osg_qi[1] = (float32_t)(0.0);
    sogi_tustin->osg_qi[2] = (float32_t)(0.0);

}

// =============================================================================
/*******************************************************************************
 函数名称：   void SOGI_Tustin_coeff_calc(SOGI_Tustin *sogi_tustin)
 功能描述：   基于Tustin变换计算SOGI离散滤波器系数
            实现从连续域到离散域的精确变换
 输入参数：   sogi_tustin : SOGI_Tustin结构体指针
 输出参数：   无
 返 回 值：  无
 其它说明：   a. 采用Tustin变换（双线性变换）进行离散化
          b. 自动计算二阶广义积分器的所有滤波器系数
          c. 系数计算基于预设的基波频率和采样周期

 设计原理：
     连续系统传递函数：
         同相分量：G_α(s) = kωs / (s² + kωs + ω²)
         正交分量：G_β(s) = kω² / (s² + kωs + ω²)

     Tustin变换公式：s = (2/Ts) * (z-1)/(z+1)

     离散化后得到二阶IIR滤波器通用形式：
         H(z) = (b₀ + b₁z⁻¹ + b₂z⁻²) / (1 - a₁z⁻¹ - a₂z⁻²)

     计算步骤：
         1. 计算角频率 ωn = 2π * fn
         2. 设置阻尼系数 k = 1.0（临界阻尼）
         3. 计算中间变量 osgx = 2kωnTs, osgy = (ωnTs)²
         4. 计算分母归一化系数 temp = 1/(osgx + osgy + 4)
         5. 分别计算同相分量和正交分量的滤波器系数
 系数含义：
     osg_b0, osg_b2 : 同相分量分子系数
     osg_a1, osg_a2 : 公共分母系数
     osg_qb0, osg_qb1, osg_qb2 : 正交分量分子系数
 修改日期         版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/11/12    V1.0           LW              创建
********************************************************************************/
void SOGI_Tustin_coeff_calc(SOGI_Tustin *sogi_tustin)
{
     float32_t osgx,osgy,temp, wn;

     // 计算角频率ωn = 2π * fn
     wn=PI2* sogi_tustin->fn ;

     // 设置固定阻尼系数k=1.0（临界阻尼）
     sogi_tustin->osg_coeff.osg_k=(float32_t)(1.0);

     // 计算中间变量：osgx = 2*k*ωn*Ts
     osgx = (float32_t)(2.0f*1.0f*wn*sogi_tustin->delta_t);
     sogi_tustin->osg_coeff.osg_x=(float32_t)(osgx);

     // 计算中间变量：osgy = (ωn*Ts)^2
     osgy = (float32_t)(wn*sogi_tustin->delta_t*wn*sogi_tustin->delta_t);
     sogi_tustin->osg_coeff.osg_y=(float32_t)(osgy);

     // 计算分母项：temp = 1/(osgx + osgy + 4)
     temp = (float32_t)1.0/(osgx+osgy+4.0f);

     // 计算同相分量系数
     sogi_tustin->osg_coeff.osg_b0=((float32_t)osgx*temp);                        // z⁰项系数
     sogi_tustin->osg_coeff.osg_b2=((float32_t)(-1.0f)*sogi_tustin->osg_coeff.osg_b0);   // z⁻²项系数

     // 计算分母系数
     sogi_tustin->osg_coeff.osg_a1=((float32_t)(2.0*(4.0f-osgy))*temp);   // z⁻¹项系数
     sogi_tustin->osg_coeff.osg_a2=((float32_t)(osgx-osgy-4)*temp);       // z⁻²项系数

     // 计算正交分量系数
     sogi_tustin->osg_coeff.osg_qb0=((float32_t)(1.0f*osgy)*temp);               // z⁰项系数
     sogi_tustin->osg_coeff.osg_qb1=(sogi_tustin->osg_coeff.osg_qb0*(float32_t)(2.0));  // z⁻¹项系数
     sogi_tustin->osg_coeff.osg_qb2=sogi_tustin->osg_coeff.osg_qb0;                     // z⁻²项系数
}


// =============================================================================
/*******************************************************************************
 函数名称：   void SOGI_Tustin_config(SOGI_Tustin *sogi_tustin,
                                    float32_t acFreq,
                                    float32_t isrFrequency)
 功能描述：   配置基于Tustin变换的SOGI系统参数
           初始化滤波器结构体并计算离散化系数
 输入参数：   sogi_tustin  : SOGI_Tustin结构体指针
           acFreq      : 电网标称频率（Hz）
           isrFrequency: 中断服务例程频率/采样频率（Hz）
 输出参数：   无
 返 回 值：  无
 其它说明：   a. 配置SOGI系统的核心参数：基波频率和采样频率
           b. 自动计算基于Tustin变换的离散滤波器系数
           c. 必须在调用SOGI算法前完成配置
 修改日期          版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/11/12    V1.0           LW              创建
********************************************************************************/
 void SOGI_Tustin_config(SOGI_Tustin *sogi_tustin,
                          float32_t acFreq,
                          float32_t isrFrequency
                        )
{
     // 设置电网标称频率（Hz）
     sogi_tustin->fn=acFreq;

     // 计算采样周期 Ts = 1 / Fs
     sogi_tustin->delta_t=((1.0f)/isrFrequency);

     // 基于Tustin变换计算sogi_tustin离散滤波器系数
     SOGI_Tustin_coeff_calc(sogi_tustin);

}


 // =============================================================================
 /*******************************************************************************
  函数名称：   void SOGI_Tustin_alpha_beta(SOGI_Tustin *sogi_tustin,
                                        float32_t voltage_Value,
                                        float32_t current_value)
  功能描述：   基于Tustin（双线性）变换的SOGI正交信号生成算法
             生成电压和电流的α-β正交分量
  输入参数：   sogi_tustin   : SOGI_Tustin结构体指针
            voltage_Value: 交流电压瞬时采样值
            current_value: 交流电流瞬时采样值
  输出参数：   无
  返 回 值：  无
  其它说明：   a. 采用传递函数法+双线性变换离散化
            b. 同时处理电压和电流信号，生成各自的正交分量
            c. 传递函数模型：
                 同相分量：G_α(s) = kωs / (s² + kωs + ω²)
                 正交分量：G_β(s) = kω² / (s² + kωs + ω²)
  参数说明：
           k  : 阻尼系数（固定为1.0，临界阻尼）
           ω  : 目标角频率（rad/s），ω = 2π * fn
           Ts : 采样周期（由中断频率决定）
           fn : 基波频率（Hz）
  修改日期          版本号             修改人            修改内容
  -----------------------------------------------------------------------------
  2025/11/04      V1.0             LW              创建
 ********************************************************************************/

 /* ------------------------------------------------------------
  算法原理:
   *   SOGI通过二阶广义积分器产生与输入信号：
         - 同相的α分量（直接跟踪分量）
         - 滞后90°的β分量（正交分量）
   *   形成虚拟的α-β两相坐标系，为锁相环提供输入信号

   *   离散化方法：双线性变换（Tustin变换）
   *   变换公式：s = (2/Ts) * (z-1)/(z+1)

   *   结构特点:
   *       - 使用三阶历史缓冲区实现二阶IIR滤波器
   *       - 电压和电流通道独立处理，结构完全相同
   *       - 系数通过SOGI_Tustin_coeff_calc()函数预先计算

   *   差分方程形式:
   *       同相分量：y_α[n] = b0·(u[n] - u[n-2]) + a1·y_α[n-1] + a2·y_α[n-2]
   *       正交分量：y_β[n] = qb0·u[n] + qb1·u[n-1] + qb2·u[n-2]
   *                   + a1·y_β[n-1] + a2·y_β[n-2]
   * ------------------------------------------------------------ */
 void SOGI_Tustin_alpha_beta(SOGI_Tustin *sogi_tustin, float32_t voltage_Value, float32_t current_value)
{

     // 更新输入电压
     //u[0]存储当前采样值，u[1]存储前一时刻值，u[2]存储前前时刻值
     sogi_tustin->u[0] = voltage_Value;

     //----------------------------------------------------------------------
     // 正交信号发生器（sogi_tustin核心）
     //----------------------------------------------------------------------

     // 计算电压同相分量（α分量）
     // 差分方程：y_α[n] = b0·(u[n] - u[n-2]) + a1·y_α[n-1] + a2·y_α[n-2]
     sogi_tustin->osg_u[0] = (sogi_tustin->osg_coeff.osg_b0 *
                          (sogi_tustin->u[0] - sogi_tustin->u[2])) +
                          (sogi_tustin->osg_coeff.osg_a1 * sogi_tustin->osg_u[1]) +
                          (sogi_tustin->osg_coeff.osg_a2 * sogi_tustin->osg_u[2]);

     // 更新电压同相分量
     sogi_tustin->osg_u[2] = sogi_tustin->osg_u[1];  // 将上一时刻值osg_u[1]移至前前时刻位置osg_u[2]
     sogi_tustin->osg_u[1] = sogi_tustin->osg_u[0];  // 将当前时刻值osg_u[0]移至上一时刻位置osg_u[1]

     // 计算电压正交分量（β分量）
     // 差分方程：y_β[n] = qb0·u[n] + qb1·u[n-1] + qb2·u[n-2] + a1·y_β[n-1] + a2·y_β[n-2]
     sogi_tustin->osg_qu[0] = (sogi_tustin->osg_coeff.osg_qb0 * sogi_tustin->u[0]) +
                          (sogi_tustin->osg_coeff.osg_qb1 * sogi_tustin->u[1]) +
                          (sogi_tustin->osg_coeff.osg_qb2 * sogi_tustin->u[2]) +
                          (sogi_tustin->osg_coeff.osg_a1 * sogi_tustin->osg_qu[1]) +
                          (sogi_tustin->osg_coeff.osg_a2 * sogi_tustin->osg_qu[2]);

     // 更新电压正交分量
     sogi_tustin->osg_qu[2] = sogi_tustin->osg_qu[1]; // 将上一时刻值osg_qu[1]移至前前时刻位置osg_qu[2]
     sogi_tustin->osg_qu[1] = sogi_tustin->osg_qu[0]; // 将当前时刻值osg_qu[0]移至上一时刻位置osg_qu[1]

     // 更新输入电压
     sogi_tustin->u[2] = sogi_tustin->u[1]; // 将上一时刻输入值u[1]移至前前时刻位置u[2]
     sogi_tustin->u[1] = sogi_tustin->u[0]; // 将当前时刻输入值u[0]移至上一时刻位置u[1]

     // 更新输入电流
     //u[0]存储当前采样值，u[1]存储前一时刻值，u[2]存储前前时刻值
     sogi_tustin->i[0] = current_value;

     // 计算电流同相分量（α分量）
     // 差分方程：y_α[n] = b0·(u[n] - u[n-2]) + a1·y_α[n-1] + a2·y_α[n-2]
     sogi_tustin->osg_i[0] = (sogi_tustin->osg_coeff.osg_b0 *
                          (sogi_tustin->i[0] - sogi_tustin->i[2])) +
                          (sogi_tustin->osg_coeff.osg_a1 * sogi_tustin->osg_i[1]) +
                          (sogi_tustin->osg_coeff.osg_a2 * sogi_tustin->osg_i[2]);

     // 更新电流同相分量
     sogi_tustin->osg_i[2] = sogi_tustin->osg_i[1]; // 将上一时刻值osg_i[1]移至前前时刻位置osg_i[2]
     sogi_tustin->osg_i[1] = sogi_tustin->osg_i[0]; // 将当前时刻值osg_i[0]移至上一时刻位置osg_i[1]

     // 计算电流正交分量（β分量）
     // 差分方程：y_β[n] = qb0·u[n] + qb1·u[n-1] + qb2·u[n-2] + a1·y_β[n-1] + a2·y_β[n-2]
     sogi_tustin->osg_qi[0] = (sogi_tustin->osg_coeff.osg_qb0 * sogi_tustin->i[0]) +
                          (sogi_tustin->osg_coeff.osg_qb1 * sogi_tustin->i[1]) +
                          (sogi_tustin->osg_coeff.osg_qb2 * sogi_tustin->i[2]) +
                          (sogi_tustin->osg_coeff.osg_a1 * sogi_tustin->osg_qi[1]) +
                          (sogi_tustin->osg_coeff.osg_a2 * sogi_tustin->osg_qi[2]);

     // 更新电流正交分量
     sogi_tustin->osg_qi[2] = sogi_tustin->osg_qi[1];  // 将上一时刻值osg_qi[1]移至前前时刻位置osg_qi[2]
     sogi_tustin->osg_qi[1] = sogi_tustin->osg_qi[0];  // 将当前时刻值osg_qi[0]移至上一时刻位置osg_qi[1]


     // 更新输入电流
     sogi_tustin->i[2] = sogi_tustin->i[1]; // 将上一时刻输入值i[1]移至前前时刻位置i[2]
     sogi_tustin->i[1] = sogi_tustin->i[0]; // 将当前时刻输入值i[0]移至上一时刻位置i[1]

     //输出赋值
     str_alpha_beta.wValpha = sogi_tustin->osg_u[0];  // 电压α分量
     str_alpha_beta.wVbeta  = sogi_tustin->osg_qu[0];  // 电压β分量
     str_alpha_beta.wIalpha = sogi_tustin->osg_i[0];   // 电流α分量
     str_alpha_beta.wIbeta  = sogi_tustin->osg_qi[0];   // 电流β分量
}


 // ---------------------------------
 // End of File
 // ---------------------------------



