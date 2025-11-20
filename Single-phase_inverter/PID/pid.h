/*
 * pid.h
 *
 *  Created on: 2025年9月5日
 *      Author: HP
 */

#ifndef PID_PID_H_
#define PID_PID_H_

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
//PID结构体
typedef struct
{
    float32_t wKp, wKi ,wKd; // PID控制器参数：Kp, Ki ,Kd
    float32_t wError[2];   // 误差数组：存储当前和前一次的误差值
    float32_t wOutput[2];  // 输出数组：存储当前和前一次的控制输出值
    float32_t wOutputMax;  // 输出上限
    float32_t wOutputMin;  // 输出下限
    float32_t wIncrement;  // 增量值：用于增量式PID计算时的输出增量
    float32_t wIntegral;
}str_PIDDef;



// --------------------------------------
// 全局变量声明
// --------------------------------------
extern str_PIDDef str_PID_Vd;
extern str_PIDDef str_PID_Vq;
extern str_PIDDef str_PID_Id;
extern str_PIDDef str_PID_Iq;
extern str_PIDDef PLL_PI;
// --------------------------------------
// 函数声明
// --------------------------------------
// 电压环PI控制器参数初始化
void Init_Voltage_Loop_PID(float32_t Kp,float32_t Ki,float32_t PI_MAX,float32_t PI_MIN,
                            float32_t error,float32_t Preverror,float32_t out,float32_t Prevout);
// 电流环PI控制器参数初始化
void Init_Current_Loop_PID(float32_t Kp,float32_t Ki,float32_t PI_MAX,float32_t PI_MIN,
                           float32_t error,float32_t Preverror,float32_t out,float32_t Prevout);

// 增量式PI控制器函数
float Incremental_PID(float32_t Err, str_PIDDef *pid_set);

// PLL锁相PI控制器参数初始化
void Init_PLL_PID(float32_t wn,float32_t segma,float32_t Vp,
                  float32_t PI_MAX,float32_t PI_MIN,
                     float32_t error,float32_t Preverror);

#endif /* PID_PID_H_ */
