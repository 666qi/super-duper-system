/*
 *  ControlStrategy.c
 *
 *  Created on: 2025年9月4日
 *      Author: jzyli
 */

// --------------------------------------
// 头文件
// --------------------------------------

#include "F28x_Project.h"

// --------------------------------------
// 本地变量定义
// --------------------------------------

int16_t CMP1 = 0;
int16_t CMP2 = 0;

float32_t Nomalization = 0;

float32_t SinVar = 0.0f;
float32_t CosVar = 0.0f;

// 电压
// --------------------------
float32_t Vref  = 0;
float32_t Vdref = 0;
float32_t Vqref = 0;
float32_t Vd1ref= 0;
float32_t Vq1ref= 0;

// 电流
// --------------------------
float32_t Iref  = 0;
float32_t Idref = 0;
float32_t Iqref = 0;
float32_t Idref1 = 0;
float32_t Iqref1 = 0;

//调试所用变量
float32_t err = 0;
float32_t errV = 0;
float32_t x1= 0;
float32_t phase_error=0;
float32_t V_m;

// --------------------------------------
// 函数定义
// --------------------------------------
// =============================================================================
/*******************************************************************************
 函数名称：    void Feedback_control(void)
 功能描述：    实现环路控制
 输入参数：    无
 输出参数：    无
 返 回 值：   无
 其它说明：
 修改日期         版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/10/21    V1.0          LW              创建
********************************************************************************/
/* ------------------------------------------------------------
 *
 * 函数功能
 *
 * 1. 计算当前角度的正弦和余弦值
 * 2. 计算当前α、β轴上的分量
 * 3. Park变换，将α、β轴上的分量转换为d、q轴上的分量
 * 4. 获得当前的弧度
 * 5. 环路控制选择
 * 6. Park逆变换，将d、q轴上的分量转换为α、β轴上的分量
 * 7. PWM调制信号生成
 * 8. 更新占空比
 *
 * ------------------------------------------------------------
 * * ------------------------------------------------------------
 * Park变换：两相（α-β）静止坐标系到（d-q）同步旋转坐标系
 *
 *           -            -
 *           | cosθ  sinθ |
 *  2s to 2r |            |
 *           |-sinθ  cosθ |
 *           -            -
 * ------------------------------------------------------------
    α、β坐标系到d、q坐标系的变换公式为：
    -----------------------------------
    d = α*cosθ  + β*sinθ
    q = -α*sinθ + β*cosθ
 *  ------------------------------------------------------------
 *
 *  ------------------------------------------------------------
 * Park逆变换：（d-q）同步旋转坐标系到两相（α-β）静止坐标系
 *
 *           -             -
 *           | cosθ  -sinθ |
 *  2r to 2s |             |
 *           | sinθ   cosθ |
 *           -             -
 *
 *  ------------------------------------------------------------
    d、q坐标系到α、β坐标系的变换公式为：
    α=d*cosθ - q*sinθ
    β=d*sinθ + q*cosθ
 ------------------------------------------------------------ */

void Feedback_control(void)
{

    // 1. 计算当前角度的正弦和余弦值
    // -------------------------------
    SinVar = sinf(g_Theta+PI/2);         //加PI/2是为了使锁相时theta值在采样电压归零处归零
    CosVar = cosf(g_Theta+PI/2);

    // 2. 计算当前α、β轴上的分量
    // -------------------------------

    //延时法
    #if(Phase_locked_mode == Phase_Delay )
        get_alpha_beta();
    #endif
    //SOGI法
    #if(Phase_locked_mode == Phase_SOGI )
        SOGI_alhfa_beta();
    #endif
    //SOGI_传递函数法
    #if(Phase_locked_mode == Phase_SOGI_Tustin)
    SOGI_Tustin_alpha_beta(&sogi_tustin, g_VoInst,g_LiInst);
    #endif
    //SOGI_状态方程法
    #if(Phase_locked_mode == Phase_SOGI_Statespace)
    SOGI_StateSpace_alpha_beta(&sogi_statespace, g_VoInst,g_LiInst);
    #endif

    // 3.Park变换
    // -------------------------------
    str_d_q.wVd =  str_alpha_beta.wValpha * CosVar + str_alpha_beta.wVbeta * SinVar;   //将电压α、β转换成d、q
    str_d_q.wVq = -str_alpha_beta.wValpha * SinVar + str_alpha_beta.wVbeta * CosVar;

    str_d_q.wId =  str_alpha_beta.wIalpha * CosVar + str_alpha_beta.wIbeta * SinVar;   //将电流α、β转换成d、q
    str_d_q.wIq = -str_alpha_beta.wIalpha * SinVar + str_alpha_beta.wIbeta * CosVar;

    // 锁相
    // -------------------------------
    #if (PLL_mode==Delay_Pll )
          Delay_PLL();                                  //延时锁相
    #endif
    #if(PLL_mode==SOGI_Pll )
          SOGI_PLL();                                  //SOGI锁相
    #endif

    // 锁相环(PLL)相位差测量
    // -------------------------------
    /* 原理说明：
     * 1. 电压矢量：Vα=Vm·sin(ωt), Vβ=Vm·cos(ωt)
     * 2. 经Park变换得Vq：Vq = -Vm·sin(Δθ) ≈ -Vm·Δθ (小角度近似)
     * 3. 相位误差：Δθ ≈ -Vq/Vd = -Vq/Vm
     */
    // --------------------------------------------------------------------
     V_m= fabs(str_d_q.wVd);  // 使用d轴分量作为幅值近似

     if (V_m > 0.001f)    // 避免除零
     {
     phase_error =-str_d_q.wVq / V_m;  // Δθ = -Vq/Vm
     }

//     4.获得当前的弧度
//     -------------------------------
//      g_Omega = PI2*50;         // 设置角速度(100π rad/s，对应50Hz)
//      g_Theta += g_Omega * Ts;   // 角度积分: 角速度 × 采样时间 = 角度增量
//
//
//      // g_Theta 角度范围限制(0 - 2π)，防止积分溢出
//      // -------------------------
//      if (g_Theta > PI2)
//        g_Theta -= PI2;   // 超过 2π 时减去 2π
//      if (g_Theta < 0)
//        g_Theta += PI2;   // 小于 0 时加上 2π

    // 5.环路控制选择
    // -----------------------------------------------

    // 5.1 开环控制
    // -----------------------------------------------
    #if(loop_mode == Open_loop)

    // 直接设定参考值
    // -------------------------
    Vdref = OPEN_VREF_PK;     // 设置d轴参考电压(开环给定值)
    Vqref = 0;                // 设置q轴参考电压(通常设为0)

    #endif

    // 5.2 电压环控制
    // -----------------------------------------------
    #if(loop_mode == Voltage_loop)

    //设定d、q轴上电压环的参考值
    // -------------------------
    Vdref = V_VREF_PK;     // 设置d轴参考电压
    Vqref = 0;            // 设置q轴参考电压(通常设为0)

    Idref=Incremental_PID(Vdref - str_d_q.wVd,  &str_PID_Vd);   // 控制d轴电压
    Iqref=Incremental_PID(Vqref - str_d_q.wVq,  &str_PID_Vq);   // 控制q轴电压
    err=Vqref - str_d_q.wVq;
    #endif

    // 5.3 电流环控制
    // -----------------------------------------------
    #if(loop_mode == Current_loop)

    //设定d、q轴上电流环的参考值
    // -------------------------
    Idref = I_IREF_PK;     // 设置d轴参考电压
    Iqref = 0;            // 设置q轴参考电压(通常设为0)

    Vdref=Incremental_PID(Idref - str_d_q.wId,  &str_PID_Id);   // 控制d轴电压
    Vqref=Incremental_PID(Iqref - str_d_q.wIq,  &str_PID_Iq);   // 控制q轴电压
    err=Iqref - str_d_q.wIq;
    #endif

     // 5.4 电压电流双环控制
     // -----------------------------------------------
     #if(loop_mode == Dual_loop)

     //设定d、q轴上电压环的参考值
     // -------------------------
     Vdref = DUAL_VREF_PK;     // 设置d轴参考电压
     Vqref = 0;               // 设置q轴参考电压(通常设为0)

     Idref=Incremental_PID(Vdref - str_d_q.wVd,  &str_PID_Vd);   // 控制d轴电压
     Iqref=Incremental_PID(Vqref - str_d_q.wVq,  &str_PID_Vq);   // 控制q轴电压
     Idref1=Idref*0.1;
     Iqref1=Iqref*0.1;
     Vd1ref=Incremental_PID(Idref1 - str_d_q.wId,  &str_PID_Id);   // 控制d轴电流
     Vq1ref=Incremental_PID(Iqref1 - str_d_q.wIq,  &str_PID_Iq);   // 控制q轴电流
     err=Idref - str_d_q.wId;
     errV=Iqref - str_d_q.wIq;
     #endif

    // 6. 反Park变换
    // -----------------------------------------------

    // 6.1 开环控制反Park变换
    // -----------------------------------------------
    #if(loop_mode == Open_loop)

        Vref = Vdref * CosVar - Vqref * SinVar;

        // 归一化参考值
        Nomalization = V_NormCoeff * Vref;    // 归一化系数乘以参考电压

    #endif
    // 6.2 电压环控制反Park变换
    // -----------------------------------------------
    #if(loop_mode == Voltage_loop)

        Vref  =  Idref * CosVar - Iqref * SinVar;      // dq反变换出alpha实际电流波形，用于单电压环

        // 归一化参考值
        Nomalization = V_NormCoeff * Vref;           // 将输出实际电流归一化

    #endif
    // 6.3 电流环控制反Park变换
    // -----------------------------------------------
    #if(loop_mode == Current_loop)

        Iref  =  Vdref * CosVar - Vqref * SinVar;      // dq反变换出alpha实际电压波形，用于单电流环

        // 归一化参考值
        Nomalization = I_NormCoeff * Iref;           // 将输出实际电流归一化

    #endif
    // 6.2 电压外环电流内环控制反Park变换
    // -----------------------------------------------
    #if(loop_mode == Dual_loop)

        Vref  = Vd1ref* CosVar - Vq1ref * SinVar;      // dq反变换出alpha实际电流波形，用于单电压环

        // 归一化参考值
        Nomalization = I_NormCoeff * Vref;           // 将输出实际电流归一化

    #endif

        //限幅
        if (Nomalization > 1.0f) Nomalization = 1.0f;
        if (Nomalization < -1.0f) Nomalization = -1.0f;

    // 7. PWM调制信号生成
    // -----------------------------------------------
    // 根据宏定义选择调制方式
    #if (Spwm_mode == Unipolar_DB) // 单极性倍频调制

        SPWM_Unipolar_DB(Nomalization, &CMP1, &CMP2);

    #endif

    // 8. 更新PWM比较寄存器
    // -----------------------------------------------
    EPwm1Regs.CMPA.bit.CMPA = CMP1;  // 更新ePWM1模块比较器A值
    EPwm2Regs.CMPA.bit.CMPA = CMP2;  // 更新ePWM2模块比较器A值

}


// ---------------------------------
// End of File
// ---------------------------------
