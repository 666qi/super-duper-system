/*
 * pid.c
 *
 *  Created on: 2025年9月5日
 *      Author: LW
 */

// --------------------------------------
// 头文件
// --------------------------------------
#include "F28x_Project.h"


// --------------------------------------
// 变量定义
// --------------------------------------


// 变量声明
// -------------------------------------------------------------------------
str_PIDDef str_PID_Vd;
str_PIDDef str_PID_Vq;
str_PIDDef str_PID_Id;
str_PIDDef str_PID_Iq;
str_PIDDef str_PID_Iq;
str_PIDDef PLL_PI;
// --------------------------------------
// 函数定义
// -------------------------------------

// =============================================================================
/*******************************************************************************
 函数名称：    float Incremental_PID(float32_t Err, str_PIDDef *pid_set)
 功能描述：    增量式PI控制器函数（无微分项）
 输入参数：    Err       当前控制误差（设定值 - 实际测量值）
              pid_set   PID控制器结构体指针
 输出参数：    无
 返 回 值：    float  经过限幅处理后的控制输出值
 其它说明：    1. 此为增量式PI控制器实现，仅包含比例和积分项
              2. 每次调用会更新控制器内部状态（误差和输出历史值）
              3. 输出值经过上下限幅处理，防止积分饱和
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/09/05    V1.0           LW              创建
********************************************************************************/
/* ------------------------------------------------------------
 * 算法实现说明：
 * 1. 更新当前误差值：e(k) = Err
 * 2. 计算比例项：P = Kp × (e(k) - e(k-1))
 * 3. 计算积分项：I = Ki × e(k)
 * 4. 计算输出增量：Δu(k) = P + I
 * 5. 计算当前输出: u(k) = u(k-1) + Δu(k)
 * 5. 对输出值进行限幅处理: u(k) = clamp(u(k), OutputMin, OutputMax)
 * 6. 更新历史状态: e(k-1) = e(k), u(k-1) = u(k)
 * 7. 返回当前控制输出值：u(k)
 * 增量式PID公式：
 *              Δu(k) = Kp×[e(k)-e(k-1)] + Ki×e(k) + Kd×[e(k)-2e(k-1)+e(k-2)]
 *              u(k) = u(k-1) + Δu(k)
 ------------------------------------------------------------ */
float Incremental_PID(float32_t Err, str_PIDDef *pid_set)
{


    float P_temp, I_temp;     // 临时变量：比例项和积分项计算结果

    pid_set->wError[0] = Err;  // 更新当前误差值  e(k) = Err

    // 计算比例项：P = Kp × (e(k) - e(k-1))
    P_temp = pid_set->wKp * (pid_set->wError[0] - pid_set->wError[1]);

    // 计算积分项：I = Ki × e(k)
    I_temp = pid_set->wKi * pid_set->wError[0];

    // 计算当前输出: u(k) = u(k-1) + Δu(k)
    pid_set->wIncrement = P_temp + I_temp;
    pid_set->wOutput[0] = pid_set->wIncrement + pid_set->wOutput[1];

    // 输出限幅处理 - 防止控制器饱和
    if (pid_set->wOutput[0] > pid_set->wOutputMax)
        pid_set->wOutput[0] = pid_set->wOutputMax;

    if (pid_set->wOutput[0] < pid_set->wOutputMin)
        pid_set->wOutput[0] = pid_set->wOutputMin;

    // 更新历史状态，为下一次计算做准备
    pid_set->wOutput[1] = pid_set->wOutput[0];      // u(k-1) = u(k)
    pid_set->wError[1]  = pid_set->wError[0];       // e(k-1) = e(k)

    return (float32_t) pid_set->wOutput[0];       // 返回当前控制输出值 u(k)
}

// =============================================================================
/*******************************************************************************
 函数名称：    void PID_Voltage_Loop_Init(float32_t Kp,float32_t Ki,float32_t PI_MAX,float32_t PI_MIN,
                                          float32_t error,float32_t Preverror)
 功能描述：    电压环PI控制器参数初始化
 输入参数：    Kp         比例系数
              Ki         积分系数（将乘以采样时间Ts进行离散化）
              PI_MAX     输出上限值
              PI_MIN     输出下限值
              error      初始误差值
              Preverror  前次误差值
 输出参数：    无
 返 回 值：    无
 其它说明：    1. 同时初始化d轴和q轴PI控制器，且两者参数相同
              2. 积分系数Ki会乘以采样时间Ts进行离散化处理
              3. 适用于电压环控制的PI参数初始化
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/09/05    V1.0           LW              创建
********************************************************************************/
/* ------------------------------------------------------------
 * 初始化步骤说明：
 * 1. 设置d轴和q轴控制器的比例系数Kp
 * 2. 设置离散化后的积分系数Ki = Ki × Ts（Ts为采样周期）
 * 3. 初始化当前误差值error[0]
 * 4. 初始化前次误差值error[1]
 * 5. 设置输出上下限值（OutputMax和OutputMin）
 *
 * 注意：d轴和q轴控制器使用相同的参数配置
 ------------------------------------------------------------ */
void Init_Voltage_Loop_PID(float32_t Kp,float32_t Ki,float32_t PI_MAX,float32_t PI_MIN,
                            float32_t error,float32_t Preverror,float32_t out,float32_t Prevout)
{
    // 设置d轴和q轴控制器的kp系数
    str_PID_Vd.wKp = Kp;
    str_PID_Vq.wKp = str_PID_Vd.wKp ;              // q轴使用与d轴相同的比例系数

    // 设置离散化后的ki系数（Ki × 采样周期Ts）
    str_PID_Vd.wKi = Ki * Ts;
    str_PID_Vq.wKi = str_PID_Vd.wKi;               // q轴使用与d轴相同的积分系数

    // 初始化当前误差值
    str_PID_Vd.wError[0] = error;
    str_PID_Vq.wError[0] = str_PID_Vd.wError[0];   // q轴使用与d轴相同的当前误差

    // 初始化前次误差值
    str_PID_Vd.wError[1] = Preverror;
    str_PID_Vq.wError[1] = str_PID_Vd.wError[1];   // q轴使用与d轴相同的前次误差

    // 初始化当前输出值
    str_PID_Vd.wOutput[0] = out;
    str_PID_Vq.wOutput[0] = str_PID_Vd.wOutput[0]; // q轴使用与d轴相同的当前输出

    // 初始化前次输出值
    str_PID_Vd.wOutput[1] = Prevout;
    str_PID_Vq.wOutput[1] = str_PID_Vd.wOutput[1]; // q轴使用与d轴相同的前次输出

    // 设置输出上限值
    str_PID_Vd.wOutputMax = PI_MAX;
    str_PID_Vq.wOutputMax = str_PID_Vd.wOutputMax; // q轴使用与d轴相同的输出上限

    // 设置输出下限值
    str_PID_Vd.wOutputMin = PI_MIN;
    str_PID_Vq.wOutputMin = str_PID_Vd.wOutputMin; // q轴使用与d轴相同的输出下限
}


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_Current_Loop_PID(float32_t Kp,float32_t Ki,float32_t PI_MAX,float32_t PI_MIN,
                           float32_t error,float32_t Preverror,
                           float32_t out,float32_t Prevout)
 功能描述：    电流环PI控制器参数初始化
 输入参数：
              Kp         比例系数
              Ki         积分系数（将乘以采样时间Ts进行离散化）
              PI_MAX     输出上限值
              PI_MIN     输出下限值
              error      初始误差值
              Preverror  前次误差值
              out        当前输出值
              Prevout    前次输出值
 输出参数：    无
 返 回 值：    无
 其它说明：    1. 同时初始化d轴和q轴PI控制器，且两者参数相同
              2. 积分系数Ki会乘以采样时间Ts进行离散化处理
              3. 适用于电流环控制的PI参数初始化
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/09/05    V1.0           LW              创建
********************************************************************************/
/* ------------------------------------------------------------
 * 初始化步骤说明：
 * 1. 设置d轴和q轴控制器的比例系数Kp
 * 2. 设置离散化后的积分系数Ki = Ki × Ts（Ts为采样周期）
 * 3. 初始化当前误差值error[0]
 * 4. 初始化前次误差值error[1]
 * 5. 设置输出上下限值（OutputMax和OutputMin）
 *
 * 注意：d轴和q轴控制器使用相同的参数配置
 * 电流环作为内环控制器，需要比电压环更快的响应速度
 ------------------------------------------------------------ */
void Init_Current_Loop_PID(float32_t Kp,float32_t Ki,float32_t PI_MAX,float32_t PI_MIN,
                           float32_t error,float32_t Preverror,
                           float32_t out,float32_t Prevout)
{
    // 设置d轴和q轴控制器的kp系数
    str_PID_Id.wKp = Kp;
    str_PID_Iq.wKp = str_PID_Id.wKp;               // q轴使用与d轴相同的比例系数

    // 设置离散化后的ki系数（Ki × 采样周期Ts）
    str_PID_Id.wKi = Ki * Ts;
    str_PID_Iq.wKi = str_PID_Id.wKi;               // q轴使用与d轴相同的积分系数

    // 初始化当前误差值
    str_PID_Id.wError[0] = error;
    str_PID_Iq.wError[0] = str_PID_Id.wError[0];   // q轴使用与d轴相同的当前误差

    // 初始化前次误差值
    str_PID_Id.wError[1] = Preverror;
    str_PID_Iq.wError[1] = str_PID_Id.wError[1];    // q轴使用与d轴相同的前次误差

    // 初始化当前输出值
    str_PID_Id.wOutput[0] = out;
    str_PID_Iq.wOutput[0] = str_PID_Id.wOutput[0]; // q轴使用与d轴相同的当前输出

    // 初始化前次输出值
    str_PID_Id.wOutput[1] = Prevout;
    str_PID_Iq.wOutput[1] = str_PID_Id.wOutput[1]; // q轴使用与d轴相同的前次输出

    // 设置输出上限值
    str_PID_Id.wOutputMax = PI_MAX;
    str_PID_Iq.wOutputMax = str_PID_Id.wOutputMax; // q轴使用与d轴相同的输出上限

    // 设置输出下限值
    str_PID_Id.wOutputMin = PI_MIN;
    str_PID_Iq.wOutputMin = str_PID_Id.wOutputMin; // q轴使用与d轴相同的输出下限
}

// =============================================================================
/*******************************************************************************
 函数名称：    void Init_PLL_PI_PID(float32_t w,float32_t segma,float32_t Vp,
                                   float32_t PI_MAX,float32_tPI_MIN,
                                   float32_t error,float32_t Preverror)
 功能描述：    延时锁相环(Delay_PLL)PI控制器参数初始化
 输入参数：
              w          目标带宽（rad/s），决定系统响应速度
              segma      阻尼比
              Vp         输入电压峰值(V)
              PI_MAX     输出上限值（防止积分饱和）
              PI_MIN     输出下限值（防止积分饱和）
              error      初始误差值
              Preverror  前次误差值
 输出参数：    无
 返 回 值：    无
 其它说明：    1. 自动计算比例系数 Kp = (2*ξ*ω)/Vp
              2. 自动计算离散化积分系数 Ki = (ω²/Vp)*Ts
              3. 所有参数基于电网电压峰值Vp进行标幺化
              4. 适用于并网逆变器的锁相环控制
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/10/10    V1.0           LW              创建
********************************************************************************/
/* ------------------------------------------------------------
 * 初始化步骤说明：
 * 1. 比例系数设计：Kp = 2*阻尼比*带宽/电压峰值
 * 2. 积分系数设计：Ki = (带宽²/电压峰值)*采样周期
 * 3. 归一化处理：所有参数除以Vp实现电压标幺化
 * 4. 离散化处理：积分项已包含采样时间Ts
 *
 * 控制理论推导：
 *
 * 1. PLL开环传递函数：
 *    G(s) = (Kp·s + Ki) / s²
 *
 * 2. 闭环传递函数推导：
 *    G_close(s) = [ (Kp·s + Ki)/s² ] / [ 1 + (Kp·s + Ki)/s² ]
 *               = (Kp·s + Ki) / (s² + Kp·s + Ki)
 *
 * 3. 标准二阶系统参数代入：
 *    其中：Kp = 2·ζ·ω_n
 *         Ki = ω_n²
 *
 * 4. 最终闭环传递函数：
 *    G_CL(s) = (2·ζ·ω_n·s + ω_n²) / (s² + 2·ζ·ω_n·s + ω_n²)
 ------------------------------------------------------------ */
void Init_PLL_PID(float32_t wn,float32_t segma,float32_t Vp,
                  float32_t PI_MAX,float32_t PI_MIN,
                  float32_t error,float32_t Preverror)
{
    // 锁相环PLL的 Kp = 2*ξ*ω
    PLL_PI.wKp = (2.0f * wn * segma)/Vp;

    // 锁相环PLL的 Ki = ω*ω
    PLL_PI.wKi = ((wn * wn)/Vp) * Ts;

    // 初始化误差
    PLL_PI.wError[0] = error;           // 当前误差值
    PLL_PI.wError[1] = Preverror;       // 前次误差值

    // 输出限幅保护，防止积分饱和
    PLL_PI.wOutputMax = PI_MAX;         // 输出上限        // 上限55Hz   w=2*pi*f=345.58 rad/s
    PLL_PI.wOutputMin = PI_MIN;         // 输出下限        // 上限45Hz   w=2*pi*f=282.74 rad/s
}


// ---------------------------------
// End of File
// ---------------------------------
