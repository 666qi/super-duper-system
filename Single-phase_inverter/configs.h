/*
 * configs.h
 *
 *  Created on: 2025年9月5日
 *      Author: jzyli
 */

#ifndef CONFIGS_H_
#define CONFIGS_H_

// --------------------------------------
// 头文件
// --------------------------------------

// --------------------------------------
// 宏定义
// --------------------------------------

// =================================
// IEEE 754 浮点类型定义
// =================================

// 防止重复包含保护宏
#ifndef C2000_IEEE754_TYPES
#define C2000_IEEE754_TYPES

// TI EABI (ELF) 编译器配置
#ifdef __TI_EABI__
typedef float         float32_t;     // 32位单精度浮点类型
typedef double        float64_t;     // 64位双精度浮点类型

// TI COFF (传统) 编译器配置
#else // TI COFF
typedef float         float32_t;    // 32位单精度浮点类型
typedef long double   float64_t;    // 64位双精度浮点类型(使用long double)

#endif // __TI_EABI__
#endif // C2000_IEEE754_TYPES
// =================================


// 常数定义
// -----------------------------------
#define PI    3.14159265358   // π
#define PI2   6.28318530716   // 2π

#define Ts    0.00005          //采样频率 1/20KHz

#define V_NormCoeff   0.0222222222222222  // 电压归一化系数：1/Vdc = 1/45
#define I_NormCoeff   0.25   // 电流归一化系数：1/Ioutmax = 1/4

// 调制方式定义
// -----------------------------------
#define Bipolar      1  // 双极性调制
#define Unipolar     2  // 单极性调制
#define Unipolar_DB  3  // 单极性倍频调制

#define Spwm_mode   Unipolar_DB   //选择调制方式


// 延时 和 SOGI 只能选择一种
// -----------------------------------
#define Phase_Delay              1            // 延时法产生α、β分量
#define Phase_SOGI_Forward       2            // SOGI法产生α、β分量
#define Phase_SOGI_Tustin        3            // SOGI传递函数离散化产生α、β分量
#define Phase_SOGI_Statespace    4            // SOGI状态方程离散化产生α、β分量

#define Phase_locked_mode  Phase_SOGI_Statespace      // 选择产生α、β分量的方法

// 延时 和 SOGI 只能选择一种
// -----------------------------------
#define Delay_Pll       1           // 延时锁相
#define SOGI_Pll        2           // SOGI锁相


#define PLL_mode  SOGI_Pll      // 选择锁相的方法,建议和产生α、β分量方法保持一致


// 环路控制选择
// -----------------------------------
#define Open_loop      0
#define Voltage_loop   1
#define Current_loop   2
#define Dual_loop      3

#define loop_mode Current_loop

// 参考值参数设置
// -----------------------------------
#define OPEN_VREF_PK  (float32_t)33.9411     // 开环参考电压峰值(V)
#define V_VREF_PK     (float32_t)33.9411     // 电压环参考电压峰值(V)
#define I_IREF_PK     (float32_t)1.414       // 电流环参考电流峰值(A)
#define DUAL_VREF_PK  (float32_t)42.4264     // 电压电流双环参考电压峰值(V)

// --------------------------------------
// 自定义数据类型
// --------------------------------------

// --------------------------------------
// 全局变量声明
// --------------------------------------

// --------------------------------------
// 函数声明
// --------------------------------------



#endif /* CONFIGS_H_ */
