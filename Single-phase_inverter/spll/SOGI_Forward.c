/*
 * SOGI_Forward.c
 *
 *  Created on: 2025年11月10日
 *      Author: LW
 */

// --------------------------------------
// 头文件
// --------------------------------------

#include "F28x_Project.h"

// --------------------------------------
// 变量定义
// --------------------------------------


//α-β 变量
str_alpha_betaDef       str_alpha_beta       = {0};

//延时法

str_Delay_alpha_betaDef str_Delay_alpha_beta = {0};

//SOGI法

str_SOGI_alpha_betaDef  str_SOGI;


// --------------------------------------
// 函数定义
// --------------------------------------


// =============================================================================
/*******************************************************************************
 函数名称：   void Init_SOGI_alpha_beta(float32_t pwm_period, float32_t damping_factor,
                                     float32_t center_freq)
 功能描述：  完全初始化SOGI-αβ（二阶广义积分器）算法所有参数和状态变量
 输入参数：  pwm_period    - PWM开关周期(s)，通常为1/开关频率
         damping_factor - 阻尼系数，影响系统响应特性
         center_freq   - 中心频率(Hz)，用于谐振频率设置
 输出参数：   无
 返 回 值：  无
 其它说明：   a. 初始化SOGI正交信号发生器，包括参数、输入输出变量和中间状态
           b. 中心频率自动转换为角频率：w0 = 2π * center_freq
           c. 将所有状态变量清零，确保系统从确定状态开始运行
 修改日期        版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/11/11    V1.0           LW              创建
********************************************************************************/
/* ------------------------------------------------------------
初始化内容分类：
 1. 输入变量：电压/电流输入值清零
 2. 输出变量：αβ坐标系分量清零
 3. 参数设置：PWM周期、阻尼系数、中心频率
 4. 中间变量：计算过程中的状态变量清零
 * ------------------------------------------------------------ */
void Init_SOGI_alpha_beta(float32_t pwm_period, float32_t damping_factor,
                          float32_t center_freq)
{
    // 初始化输入变量
    str_SOGI.wUin = 0.0f;     // SOGI电压输入值(V)
    str_SOGI.wIin = 0.0f;     // SOGI电流输入值(A)

    // 初始化输出变量
    str_SOGI.wUalhfa = 0.0f;  // SOGI输出的α轴电压分量(V)
    str_SOGI.wUbeta = 0.0f;   // SOGI输出的β轴电压分量(V)
    str_SOGI.wIalhfa = 0.0f;  // SOGI输出的α轴电流分量(A)
    str_SOGI.wIbeta = 0.0f;   // SOGI输出的β轴电流分量(A)

    // 初始化参数变量
    str_SOGI.wt = pwm_period;           // PWM周期
    str_SOGI.wk = damping_factor;       // 阻尼系数
    str_SOGI.w0 = PI2 * center_freq;    // 中心角频率

    // 初始化中间变量
    str_SOGI.wx = 0.0f;        // 计算电压误差中间变量
    str_SOGI.wy = 0.0f;        // 计算电流误差中间变量
}



// =============================================================================
/*******************************************************************************
 函数名称：   void get_alpha_beta(void)
 功能描述：   通过延时法生成α-β两相正交信号
 输入参数：   无
 输出参数：   无
 返 回 值：  无
 其它说明：   a. 采用1/4周期延时法产生滞后90°的β分量
           b. 适用于50Hz电网，20kHz采样频率（400点/周期）
           c. 实现简单的正交信号分解，用于锁相环等应用
 算法原理：
     对于50Hz信号，一个周期为20ms（400个采样点）
     90°相位差对应1/4周期延时，即100个采样点的延时
     α轴：直接使用当前采样值
     β轴：使用延时100个采样点的历史值
 修改日期         版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/10/30    V1.0           LW              创建
********************************************************************************/
void get_alpha_beta(void)
{

    // 1、获取α轴的值
    // --------------------------------------------
    str_alpha_beta.wValpha = g_VoInst;  // 将交流电压的实际值赋值为α轴的电压值
    str_alpha_beta.wIalpha = g_LiInst;  // 将交流电流的实际值赋值为α轴的电流值

    // 2、获取β轴的值
    // --------------------------------------------
    // 保存当前交流电压、电流值
    str_Delay_alpha_beta.wU[str_Delay_alpha_beta.nCount_alpha] = str_alpha_beta.wValpha;
    str_Delay_alpha_beta.wI[str_Delay_alpha_beta.nCount_alpha] = str_alpha_beta.wIalpha;

    // α轴等于90°时，β轴起始标志位置位
    if (str_Delay_alpha_beta.nCount_alpha >= 100)
    {
        str_Delay_alpha_beta.nFlag_Beta = 1;  // β轴起始标志位置位
    }

    // 将α轴滞后90°时开始生成β轴
    // 当α轴=90° ，即β轴=0° ，生成β轴的值
    if (str_Delay_alpha_beta.nFlag_Beta == 1)
    {
        if (str_Delay_alpha_beta.nCount_beta >= 400)
        {
            str_Delay_alpha_beta.nCount_beta = 0; // β轴计数值清零
        }

        str_alpha_beta.wVbeta = str_Delay_alpha_beta.wU[str_Delay_alpha_beta.nCount_beta];
        str_alpha_beta.wIbeta = str_Delay_alpha_beta.wI[str_Delay_alpha_beta.nCount_beta];

        str_Delay_alpha_beta.nCount_beta++;  // 计数值累加
    }

    // 3、在周期内循环
    // --------------------------------------------
    str_Delay_alpha_beta.nCount_alpha++;           //α轴计数值累加

    if (str_Delay_alpha_beta.nCount_alpha >= 400)
    {
        //α轴计数值清零
        str_Delay_alpha_beta.nCount_alpha = 0;
    }
}


// =============================================================================
/*******************************************************************************
 函数名称：   void SOGI_alhfa_beta(void)
 功能描述：   实现二阶广义积分器(SOGI)算法，生成正交信号（α-β分量）
 输入参数：   无
 输出参数：   无
 返 回 值：   无
 其它说明：    a. 对输入电压/电流信号进行正交分解
              b. 通过反馈结构生成滞后90°的β分量
              c. 传递函数模型：
                            G_α(s) = kωs / (s² + kωs + ω²)  → α轴（同相）
                            G_β(s) = kω² / (s² + kωs + ω²)  → β轴（正交）
 参数说明：
          k  : 阻尼系数（影响系统响应速度和滤波特性）
          ω  : 目标频率（rad/s），ω=2π*f（f为基波频率）
          t  : 控制周期（通常为PWM中断周期，如20kHz对应t=50us）
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
********************************************************************************/
/* ------------------------------------------------------------
算法原理:
 *   SOGI是一种用于提取基波正序分量的滤波器结构，能够产生与输入信号
 *   同相的α分量和正交的β分量（滞后90°）
 *
 *   离散化方法:
 *   采用前向欧拉法将连续时间系统离散化
 *   离散化公式: 新值 = 旧值 + 变化率 × 时间步长
 *
 * ------------------------------------------------------------ */
void SOGI_alhfa_beta(void)
{

    // 1. 开始测量
    EALLOW;
    sogi_start_time = CpuTimer2Regs.TIM.all;
    EDIS;

    // 获取当前采样值
    str_SOGI.wUin = g_VoInst;     // 交流电压瞬时值输入
    str_SOGI.wIin = g_LiInst;    // 交流电流瞬时值输入

    // 电压SOGI计算
    str_SOGI.wx= str_SOGI.wUin - str_SOGI.wUalhfa;    // 计算电压误差(输入与α分量的差)

    // 更新电压α分量: 使用误差经过阻尼系数调整并减去β分量，再积分
    str_SOGI.wUalhfa = str_SOGI.wUalhfa+(str_SOGI.wx* str_SOGI.wk-str_SOGI.wUbeta)* str_SOGI.w0* str_SOGI.wt;
                         //  ↑旧值            ↑──           变化率           ──↑            ↑时间步长
    // 更新电压β分量: 对α分量积分得到滞后90°的β分量
    str_SOGI.wUbeta =str_SOGI.wUbeta + str_SOGI.wUalhfa * str_SOGI.w0* str_SOGI.wt;

    // 电流SOGI计算(与电压计算结构相同)
    str_SOGI.wy= str_SOGI.wIin - str_SOGI.wIalhfa;     // 计算电流误差(输入与α分量的差)

    // 更新电流α分量
    str_SOGI.wIalhfa = str_SOGI.wIalhfa+(str_SOGI.wy*str_SOGI.wk-str_SOGI.wIbeta)* str_SOGI.w0* str_SOGI.wt;
     //                    ↑旧值                   ↑──          变化率          ──↑        ↑时间步长

    // 更新电流β分量
    str_SOGI.wIbeta =str_SOGI.wIbeta + str_SOGI.wIalhfa * str_SOGI.w0* str_SOGI.wt;

    // 输出结果赋值
    str_alpha_beta.wValpha = str_SOGI.wUalhfa;    // 电压α分量输出
    str_alpha_beta.wIalpha = str_SOGI.wIalhfa;    // 电流α分量输出
    str_alpha_beta.wVbeta  = str_SOGI.wUbeta;     // 电压β分量输出
    str_alpha_beta.wIbeta  = str_SOGI.wIbeta;     // 电流β分量输出

    // 2. 结束测量（在所有操作完成后）
    EALLOW;
    sogi_end_time = CpuTimer2Regs.TIM.all;
    EDIS;

    // 3. 计算执行时间
    sogi_time_us = CalculateExecutionTime(sogi_start_time, sogi_end_time);
}

// ---------------------------------
// End of File
// ---------------------------------
