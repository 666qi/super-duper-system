/*
 * globalvars.h
 *
 *  Created on: 2025年8月27日
 *      Author: LW
 */



#ifndef GLOBALVARS_H_
#define GLOBALVARS_H_

// --------------------------------------
// 头文件
// --------------------------------------
#include "F28x_Project.h"



// --------------------------------------
// 宏定义
// --------------------------------------


// --------------------------------------
// 自定义数据类型
// --------------------------------------


// --------------------------------------
// 全局变量声明
// --------------------------------------
extern int16_t   ISR_ADCA1;   // 中断标志

extern float32_t g_VbusInst;    // 直流电压实时值
extern float32_t g_LiInst;      // 电感电流实时值
extern float32_t g_VoInst;      // 输出电压实时值

extern float32_t g_Theta ;      // 旋转角度(rad)，用于Park和反Park变换
extern float32_t g_Omega ;      // 角频率(rad/s)，ω = 2*PI*f

extern float32_t time_start;
extern float32_t time_end;
extern float32_t exec_time_us;

extern float32_t sogi_start_time;
extern float32_t sogi_end_time;
extern float32_t sogi_time_us;
// --------------------------------------
// 函数声明
// --------------------------------------
extern void InitGlobalVariable(void);

#endif/* VARIABLE_H_ */
