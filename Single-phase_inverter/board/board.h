/*
 * board.h
 *
 *  Created on: 2025年8月27日
 *      Author: jzy
 */

#ifndef BOARD_H_
#define BOARD_H_

// --------------------------------------
// 头文件
// --------------------------------------
#include "F28x_Project.h"


// --------------------------------------
// 宏定义
// --------------------------------------

//
// 系统相关参数
//
// --------------------------------------
#define CPU_CLOCK_FREQ     100000000    // CPU 时钟频率 100*1000000=100M
#define ISR_FREQ           20000        // 中断频率 20*1000=20K


// ePWM 模块相关参数
// --------------------------------------

#define EPWM1_CLOCK_FREQ   100000000    // EPWM1 时钟频率 100*1000000=100M
#define EPWM1_SW_FREQ      20000        // EPWM1 开关频率 20*1000=20K

#define EPWM2_CLOCK_FREQ   100000000    // EPWM2 时钟频率 100*1000000=100M
#define EPWM2_SW_FREQ      20000        // EPWM2 开关频率 20*1000=20K

// Time Base Period Register
// --------------------------------------

#define EPWM1_TIMER_TBPRD       (EPWM1_CLOCK_FREQ/EPWM1_SW_FREQ)    // Period register = 5000
#define EPWM1_TIMER_TBPRD_HALF  (EPWM1_CLOCK_FREQ/EPWM1_SW_FREQ/2)  // 2500

#define EPWM2_TIMER_TBPRD       (EPWM1_CLOCK_FREQ/EPWM2_SW_FREQ)    // Period register = 5000
#define EPWM2_TIMER_TBPRD_HALF  (EPWM1_CLOCK_FREQ/EPWM2_SW_FREQ/2)  // 2500

// Compare A Register
// --------------------------------------

#define EPWM1_CMPA0    0
#define EPWM1_CMPB0    0

#define EPWM2_CMPA0    0
#define EPWM2_CMPB0    0

// --------------------------------------
// 死区时间 = EPWMx_DB × (1/EPWMx_CLOCK_FREQ)
// 200 ns=20×(1/100MHz)
// 详见
// TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1924

#define EPWM1_DB   20  // 死区时间 200ns
#define EPWM2_DB   20  // 死区时间 200ns

//
// 硬件相关参数
//

// 采样通道数量
// --------------------------------------
#define VDC_SOC_COUNT   (float32_t)(0.2)   // 1/5 直流电压采样共使用 5 个通道
#define VAC_SOC_COUNT   (float32_t)(0.2)   // 1/5 交流电压采样共使用 5 个通道
#define IAC_SOC_COUNT   (float32_t)(0.2)   // 1/5 交流电流采样共使用 5 个通道


// 12位 ADC 缩放因子
// -----------------------------------
// 1/4096
// -----------------------------------
#define ADC_PU_SCALE_FACTOR        (float32_t)(0.000244140625)

// (1/4096)*3.3
#define ADC_PU_SCALE_FACTOR_3V3    (float32_t)(0.0008056640625)

// (1/4096)*3.3/5
#define ADC_PU_SCALE_FACTOR_3V3_5  (float32_t)(0.0001611328125)


// 直流电压
// --------------------------------------
// VDC = VBUS_SCALE_FACTOR * (VDC_FB - VBUS_OFFSET)
// --------------------------------------
// 1、直流电压偏置
// --------------------------------------
#define VBUS_OFFSET   (float32_t)(0.005944200130953989419749108) // 实际校准

// 2、直流电压缩放因子
// --------------------------------------
// (3.24K + 47K + 47K) / 3.24K = 30.0123 理论值 详见原理图
// --------------------------------------
#define VBUS_SCALE_FACTOR  (float32_t)(28.9089213552999689151383276344420) // 实际校准

// 交流电压
// --------------------------------------
// VAC = VO_SCALE_FACTOR * (VAC_FB - VO_OFFSET)
// --------------------------------------
// 1、交流电压偏置
// --------------------------------------
#define VO_OFFSET          (float32_t)(1.6625)  // 实际校准

// 2、交流电压缩放因子
// --------------------------------------
// 24240 / 510 = 47.529 理论值 详见原理图
#define VO_SCALE_FACTOR    (float32_t)(46.77)  // 实际校准     电压环46.97

// 电感电流
// --------------------------------------
// VLi = LI_SCALE_FACTOR * (VLi_FB - LI_OFFSET)
// 1、电感电流偏置
// --------------------------------------
#define LI_OFFSET          (float32_t)(2.503)   // 实际校准

// 2、电感电流缩放因子
// --------------------------------------
// 1A = 0.1041  1 / 0.1041 = 9.606 理论值 详见原理图
#define LI_SCALE_FACTOR    (float32_t)(9.99)  // 实际校准


// --------------------------------------
// 自定义变量
// --------------------------------------

// --------------------------------------
// 全局变量声明
// --------------------------------------

// --------------------------------------
// 函数声明
// --------------------------------------

void FloatToByte(float32_t floatNum,uint16_t byteArry[]);
void sendVOFAtail(void);
void sendVOFAonechannel(float32_t floatNum,uint16_t byteArry[]);
void InitDebug(void);
void InitBoard(void);


#endif /* BOARD_BOARD_H_ */
