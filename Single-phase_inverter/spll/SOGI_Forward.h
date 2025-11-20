/*
 * SOGI_Forward.h
 *
 *  Created on: 2025年11月10日
 *      Author: LW
 */

#ifndef SPLL_SOGI_FORWARD_H_
#define SPLL_SOGI_FORWARD_H_

// --------------------------------------
// 头文件
// --------------------------------------

// --------------------------------------
// 宏定义
// --------------------------------------

// --------------------------------------
// 自定义变量
// --------------------------------------

// 保存 α轴 β轴 的数据
typedef struct
{
    float32_t wValpha; // α轴电压分量（单位：V）
    float32_t wIalpha; // α轴电流分量（单位：A）

    float32_t wVbeta;  // β轴电压分量（单位：V）
    float32_t wIbeta;  // β轴电流分量（单位：A）

}str_alpha_betaDef;


typedef struct
{
    int16_t nCount_alpha ;  // α轴周期计数值
    int16_t nCount_beta  ;  // β轴周期计数值
    int16_t nFlag_Beta   ;  // α轴90°时 β轴起始标志位

    // 保存交流电压值，400表示 PWM 频率 20KHz 与电网频率 50Hz 的比值
    float32_t wU[400] ; // 400= 20KHz/50Hz

    // 保存交流电流值，400表示 PWM 频率 20KHz 与电网频率 50Hz 的比值
    float32_t wI[400] ; // 400= 20KHz/50Hz

}str_Delay_alpha_betaDef;


typedef struct
{
    float32_t wUin;                // SOGI电压输入值(V)
    float32_t wIin;                // SOGI电流输入值(A)
    float32_t wUalhfa;             // SOGI输出的α轴电压分量(V)
    float32_t wUbeta;              // SOGI输出的β轴电压分量(V)
    float32_t wUbeta1;             // SOGI中间变量，用于β轴计算
    float32_t wIalhfa;             // SOGI输出的α轴电流分量(A)
    float32_t wIbeta;              // SOGI输出的β轴电流分量(A)
    float32_t wt;                  // 控制周期(s)，对应20kHz PWM频率
    float32_t wk;                  // SOGI阻尼系数，影响响应速度和稳定性
    float32_t wx;                  // SOGI计算中间变量，电压误差
    float32_t wy;                  // SOGI计算中间变量，电流误差
    float32_t w0;                  // SOGI中心角频率

}str_SOGI_alpha_betaDef;



// --------------------------------------
// 变量声明
// --------------------------------------

//SOGI_α-β 变量
extern str_SOGI_alpha_betaDef  str_SOGI;

//α-β 变量
extern str_alpha_betaDef   str_alpha_beta;


// --------------------------------------
// 函数声明
// --------------------------------------
void Init_SOGI_alpha_beta(float32_t pwm_period, float32_t damping_factor,
                          float32_t center_freq);
void get_alpha_beta(void);
void SOGI_alhfa_beta(void);



#endif /* SPLL_SOGI_FORWARD_H_ */
