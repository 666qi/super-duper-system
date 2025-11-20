//###########################################################################
//
// FILE:    f28004x_adc.c
//
// TITLE:   F28004x ADC Support Functions.
//
//###########################################################################
//
//
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "f28004x_device.h"      // Header File Include File
#include "f28004x_examples.h"    // Examples Include File



// =============================================================================
/*******************************************************************************
 函数名称：    void SetVREF(int module, int mode, int ref)
 功能描述：    配置指定ADC模块的参考电压模式和偏移校准值
 输入参数：    a.module  ADC模块标识(0=ADCA, 1=ADCB, 2=ADCC)
             b. mode   参考电压模式(ADC_INTERNAL=内部参考, 其他=外部参考)
             c. ref    参考电压选择(ADC_VREF3P3=3.3V参考, 其他=替代参考电压)
 输出参数：    无
 返 回 值：    无
 其它说明：    a.从OTP存储器读取校准的偏移量值
             b.内部3.3V模式使用高字节偏移量，其他模式使用低字节偏移量
             c.配置模拟参考控制寄存器
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、确定OTP中的偏移量地址
 *    基地址: 0x70594
 *    每个模块偏移: 模块索引 × 6字节
 *
 * 2、根据模式选择偏移量字节
 *    内部3.3V模式: 使用OTP值的高字节(>>8)
 *    其他模式: 使用OTP值的低字节(&0xFF)
 *
 * 3、配置ADC偏移校准寄存器
 *    将选定的偏移量值写入ADCOFFTRIM寄存器
 *
 * 4、配置参考电压模式
 *    设置ANAREFxSEL选择内部/外部参考
 *    设置ANAREFx2P5SEL选择参考电压值
 *----------------------------------------------------------*/
void SetVREF(int module, int mode, int ref)
{
    Uint16 *offset, offval;                 // 定义指向OTP存储器的指针和偏移值变量

    //
    // 从OTP存储器定义偏移量地址
    //
    offset = (Uint16 *)(0x70594 + (module * 6));          // 计算指定ADC模块在OTP中的校准数据地址

    // 根据参考电压模式选择相应的偏移量值
    if((mode == ADC_INTERNAL) && (ref == ADC_VREF3P3))
    {
        offval = (*offset) >> 8;    // 内部3.3V参考模式：使用OTP值的高8位作为偏移量
    }
    else
    {
        offval = (*offset) & 0xFF;  // 所有其他模式：使用OTP值的低8位作为偏移量
    }

    //
    // 写入偏移校准值并配置参考电压模式
    //
    EALLOW;              // 允许访问受保护的寄存器
    switch(module)       // 根据ADC模块选择配置相应的寄存器
    {
        case 0:      // ADCA模块
            AdcaRegs.ADCOFFTRIM.bit.OFFTRIM = offval;                    // 设置ADCA的偏移校准值
            AnalogSubsysRegs.ANAREFCTL.bit.ANAREFASEL = mode;            // 设置ADCA参考电压模式
            AnalogSubsysRegs.ANAREFCTL.bit.ANAREFA2P5SEL = ref;          // 设置ADCA参考电压选择
            break;
        case 1:  // ADCB模块
            AdcbRegs.ADCOFFTRIM.bit.OFFTRIM = offval;  // 设置ADCB的偏移校准值
            AnalogSubsysRegs.ANAREFCTL.bit.ANAREFBSEL = mode;  // 设置ADCB参考电压模式
            AnalogSubsysRegs.ANAREFCTL.bit.ANAREFB2P5SEL = ref;  // 设置ADCB参考电压选择
            break;
        case 2:  // ADCC模块
            AdccRegs.ADCOFFTRIM.bit.OFFTRIM = offval;  // 设置ADCC的偏移校准值
            AnalogSubsysRegs.ANAREFCTL.bit.ANAREFCSEL = mode;  // 设置ADCC参考电压模式
            AnalogSubsysRegs.ANAREFCTL.bit.ANAREFC2P5SEL = ref;  // 设置ADCC参考电压选择
            break;
        default:  // 无效模块号
            break;  // 不执行任何操作
            }
            EDIS;  // 禁止访问受保护的寄存器，恢复寄存器保护
}


/* -----------------------------------------------------------------------
 * TMS320F28004x 一共有三个 3.45MSPS 12 位模数转换器 (ADC)
 * ADC 的转换周期 10.5 个 ADCCLK
 *
 * SYSCLK = 100M
 * ADCCLK = 100M/4 = 25M
 * ADC 的转换周期 10.5*(1/ADCCLK) = 10.5*4*(1/SYSCLK) =42*(1/SYSCLK) = 420ns
 *-----------------------------------------------------------------------*/


// 自己添加的函数
/* --------------------------------------------------------------
 * 用户根据自己的实际需求修改
 * --------------------------------------------------------------*/


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ADC(void)
 功能描述：    初始化三个ADC模块(ADCA, ADCB, ADCC)的基本配置
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    a.配置内部参考电压为3.3V
 *            b.设置ADC时钟分频为4分频
 *            c.配置中断在转换完成后产生
 *            d.为ADC模块上电并等待稳定
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、配置 ADC 参考源
 * ADC 参考源 1、外部参考源 2、内部参考源
 * 内部参考源 0 (0-3.3) 1(0-2.5)
 *
 * 2、配置 ADC 的时钟分频系数
 * SYSCLK = 100M
 * ADCCLK = 100M/4 = 25M
 *
 * 3、配置 ADC Interrupt Pulse Position （ ADC 中断脉冲产生的时机 ）
 * 0  ADC Interrupt Pulse begins conversion
 * 1  ADC Interrupt Pulse end of the conversion
 *
 * 4、 Power up the ADC （ ADC 模块上电 ）
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1557
 *----------------------------------------------------------*/
void Init_ADC(void){

    // 设置内部参考源 3.3 or 2.5
    //TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1553
    SetVREF(ADC_ADCA, ADC_INTERNAL, ADC_VREF3P3);    // 设置ADCA内部参考电压为3.3V
    SetVREF(ADC_ADCB, ADC_INTERNAL, ADC_VREF3P3);    // 设置ADCB内部参考电压为3.3V
    SetVREF(ADC_ADCC, ADC_INTERNAL, ADC_VREF3P3);    // 设置ADCC内部参考电压为3.3V

    EALLOW;     // 允许访问受保护的寄存器

    // 设置ADCCLK分频系数为4分频
    // 计算公式: ADCCLK = SYSCLK / (PRESCALE + 1)
   // SYSCLK = 100MHz, ADCCLK = 100MHz / 4 = 25MHz
    AdcaRegs.ADCCTL2.bit.PRESCALE = ADC_Clock_Prescaler_4;    // 设置ADCA时钟分频为4分频
    AdcbRegs.ADCCTL2.bit.PRESCALE = ADC_Clock_Prescaler_4;    // 设置ADCB时钟分频为4分频
    AdccRegs.ADCCTL2.bit.PRESCALE = ADC_Clock_Prescaler_4;    // 设置ADCC时钟分频为4分频

    // 配置中断脉冲位置为转换结束后产生
    // 可选择在转换开始或转换结束时产生中断
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = ADC_COMPLETE_END;     // ADCA转换完毕后产生中断
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = ADC_COMPLETE_END;     // ADCB转换完毕后产生中断
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = ADC_COMPLETE_END;     // ADCC转换完毕后产生中断

    // ADC 上电
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = Power_Up;               // ADCA上电
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = Power_Up;               // ADCB上电
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = Power_Up;               // ADCC上电

    EDIS;     // 禁止访问受保护的寄存器


    // 延时10ms等待ADC电源稳定
    // ADC上电后需要一定时间稳定，确保转换精度
    DELAY_US(10000);
}


// =============================================================================
/*******************************************************************************
 函数名称：    void ConfigureADCA_SOC(void)
 功能描述：    a.配置ADCA模块的启动转换(SOC)参数，用于直流电压采集
              b.配置6个SOC全部使用ADCINA6通道，由ePWM1的ADCSOCA事件触发
              c.配置ADCA中断，在SOC5完成后触发中断1
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    a.使用ADCINA6通道(J7-63)进行直流电压采集
              b.配置6个SOC实现过采样，提高测量精度
              c.采样由ePWM1模块同步触发，确保采样时刻准确性
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、配置 ADC 参考源
 * ADC 参考源 1、外部参考源 2、内部参考源
 * 内部参考源 0 (0-3.3) 1(0-2.5)
 *
 * 2、配置 ADC 的时钟分频系数
 * SYSCLK = 100M
 * ADCCLK = 100M/4 = 25M
 *
 * 3、配置 ADC Interrupt Pulse Position （ ADC 中断脉冲产生的时机 ）
 * 0  ADC Interrupt Pulse begins conversion
 * 1  ADC Interrupt Pulse end of the conversion
 *
 * 4、 Power up the ADC （ ADC 模块上电 ）
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1539
 *----------------------------------------------------------*/
void ConfigureADCA_SOC(void){
    // -----------------------------------------------
    // 直流电压采集输入
    // 开发板硬件端口 ADCINA6 J7-63
    // 软件使用过采样 SOC0 - SOC5 共 6 个 SOC
    // -----------------------------------------------

    EALLOW;     // 允许访问受保护的寄存器

    // SOC0 配置
    // -----------------------------------------------
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;             // 选择 ADCSOC0 的输入为 ADCINA6
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC0由ePWM1的ADCSOCA事件触发
    AdcaRegs.ADCINTSOCSEL1.bit.SOC0 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC0

    // SOC1 配置
    // -----------------------------------------------
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;             // 选择 ADCSOC1 的输入为 ADCINA6
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC1由ePWM1的ADCSOCA事件触发
    AdcaRegs.ADCINTSOCSEL1.bit.SOC1 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC1

    // SOC2 配置
    // -----------------------------------------------
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;             // 选择 ADCSOC2 的输入为 ADCINA6
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC2由ePWM1的ADCSOCA事件触发
    AdcaRegs.ADCINTSOCSEL1.bit.SOC2 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC2

    // SOC3 配置
    // -----------------------------------------------
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;             // 选择 ADCSOC3 的输入为 ADCINA6
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC3由ePWM1的ADCSOCA事件触发
    AdcaRegs.ADCINTSOCSEL1.bit.SOC3 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC3

    // SOC4
    // -----------------------------------------------
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;            // 选择 ADCSOC4 的输入为 ADCINA6
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;  // 设置SOC4由ePWM1的ADCSOCA事件触发
    AdcaRegs.ADCINTSOCSEL1.bit.SOC4 = ADCINT_Trigger_DISABLE;    // 禁止ADC中断触发SOC4


    // SOC5
    // -----------------------------------------------
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;            // 选择 ADCSOC5 的输入为 ADCINA6
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;  // 设置SOC5由ePWM1的ADCSOCA事件触发
    AdcaRegs.ADCINTSOCSEL1.bit.SOC5 = ADCINT_Trigger_DISABLE;    // 禁止ADC中断触发SOC5

    // -----------------------------------------------
    // SYSCLK=100M  ADCCLK = 100M/4 = 25M
    // ADC采样和保持电路(S+H)由SYSCLK时钟，而ADC转换过程由ADCCLK时钟。
    //   采集时间 = (ACQPS + 1) × SYSCLK周期
    //   5个SYSCLK周期 = 5 × 10ns = 50ns
    // -----------------------------------------------
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 4;     // SOC0采样窗口: 5个SYSCLK周期 (50ns)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 4;     // SOC1采样窗口: 5个SYSCLK周期 (50ns)
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 4;     // SOC2采样窗口: 5个SYSCLK周期 (50ns)
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 4;     // SOC3采样窗口: 5个SYSCLK周期 (50ns)
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 4;     // SOC4采样窗口: 5个SYSCLK周期 (50ns)
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = 4;     // SOC5采样窗口: 5个SYSCLK周期 (50ns)

    // 突发模式配置
    // 注: 突发模式允许单个触发信号启动多个SOC转换
    AdcaRegs.ADCBURSTCTL.bit.BURSTEN = ADC_BURSTEN_DISABLE;                  // 禁用突发模式
    //AdcaRegs.ADCBURSTCTL.bit.BURSTEN = ADC_BURSTEN_ENABLE;                 // 使能  Burst Mode
    //AdcaRegs.ADCBURSTCTL.bit.BURSTTRIGSEL = ADC_BURSTTRIGSEL_ePWM7ADCSOCA; // 设置 Burst 的触发源

    // 配置 SOCx 的优先级
    AdcaRegs.ADCSOCPRICTL.bit.RRPOINTER = ADCSOC_RRPOINTER_Reset; // 循环优先级指针复位到SOC0
    AdcaRegs.ADCSOCPRICTL.bit.SOCPRIORITY = ADCSOC_PRIORITY_RR;   // 所有通道使用循环优先级模式

    // ADC_A 中断配置
    // INT1 和 INT2
    // -----------------------------------------------
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = ADC_INTxSEL_EOC5;       // ADC_A 中断1 触发源 SOC5
    //AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = ADC_INTxSEL_EOC3;     // ADC_A 中断1 触发源 SOC3

    //AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = ADC_INTxCONT_DISABLE;   // ADC_A 中断1 连续中断禁止
    //AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = ADC_INTxCONT_DISABLE;   // ADC_A 中断2 连续中断禁止

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = ADC_INTx_ENABLE;         // ADC_A 中断1 使能
    //AdcaRegs.ADCINTSEL1N2.bit.INT2E = ADC_INTx_ENABLE;         // ADC_A 中断2 使能

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = ADC_ADCINTxFLG_CLEAR;   // ADC_A中断1 标志清零
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = ADC_ADCINTxFLG_CLEAR;   // ADC_A中断2 标志清零

    // INT3 和 INT4
    // -----------------------------------------------
//    AdcaRegs.ADCINTSEL3N4.bit.INT3SEL = ADC_INTxSEL_EOC7;       // ADC_A 中断3 触发源 SOC7
//    AdcaRegs.ADCINTSEL3N4.bit.INT4SEL = ADC_INTxSEL_EOC15;      // ADC_A 中断4 触发源 SOC15
//
//    AdcaRegs.ADCINTSEL3N4.bit.INT3CONT = ADC_INTxCONT_ENABLE;   // ADC_A 中断3 连续中断使能
//    AdcaRegs.ADCINTSEL3N4.bit.INT4CONT = ADC_INTxCONT_ENABLE;   // ADC_A 中断4 连续中断使能
//
//    AdcaRegs.ADCINTSEL3N4.bit.INT3E = ADC_INTx_ENABLE;         // ADC_A 中断3 使能
//    AdcaRegs.ADCINTSEL3N4.bit.INT4E = ADC_INTx_ENABLE;         // ADC_A 中断4 使能
//
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT3 = ADC_ADCINTxFLG_CLEAR;  // ADC_A中断3 标志清零
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT4 = ADC_ADCINTxFLG_CLEAR;  // ADC_A中断4 标志清零

    EDIS;    // 禁止访问受保护的寄存器
}


// =============================================================================
/*******************************************************************************
 函数名称：    void ConfigureADCB_SOC(void)
 功能描述：    a.配置ADCB模块的启动转换(SOC)参数，用于电感电流采集
              b.配置5个SOC全部使用ADCINB6通道，由ePWM1的ADCSOCA事件触发
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    a.使用ADCINB6通道(J7-64)进行电感电流采集
              b.配置5个SOC实现过采样，提高测量精度
              c.采样由ePWM1模块同步触发，确保采样时刻准确性
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、配置 ADC 参考源
 * ADC 参考源 1、外部参考源 2、内部参考源
 * 内部参考源 0 (0-3.3) 1(0-2.5)
 *
 * 2、配置 ADC 的时钟分频系数
 * SYSCLK = 100M
 * ADCCLK = 100M/4 = 25M
 *
 * 3、配置 ADC Interrupt Pulse Position （ ADC 中断脉冲产生的时机 ）
 * 0  ADC Interrupt Pulse begins conversion
 * 1  ADC Interrupt Pulse end of the conversion
 *
 * 4、 Power up the ADC （ ADC 模块上电 ）
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1557
 *----------------------------------------------------------*/
void ConfigureADCB_SOC(void){
    // -----------------------------------------------
    // 电感电流采样
    // 开发板硬件端口 ADCINB6 J7-64
    // 软件使用过采样 SOC0 - SOC4 共 5 个 SOC
    // -----------------------------------------------

    EALLOW;      // 允许访问受保护的寄存器

    // SOC0
    // -----------------------------------------------
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;             // 选择 ADCSOC0 的输入为 ADCINB6
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC0由ePWM1的ADCSOCA事件触发
    AdcbRegs.ADCINTSOCSEL1.bit.SOC0 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC0

    // SOC1
    // -----------------------------------------------
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;             // 选择 ADCSOC1 的输入为 ADCINB6
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC1由ePWM1的ADCSOCA事件触发
    AdcbRegs.ADCINTSOCSEL1.bit.SOC1 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC1

    // SOC2
    // -----------------------------------------------
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;             // 选择 ADCSOC2 的输入为 ADCINB6
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC2由ePWM1的ADCSOCA事件触发
    AdcbRegs.ADCINTSOCSEL1.bit.SOC2 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC2

    // SOC3
    // -----------------------------------------------
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;             // 选择 ADCSOC3 的输入为 ADCINB6
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC3由ePWM1的ADCSOCA事件触发
    AdcbRegs.ADCINTSOCSEL1.bit.SOC3 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC3

    // SOC4
    // -----------------------------------------------
    AdcbRegs.ADCSOC4CTL.bit.CHSEL = ADC_CHSEL_ADCIN6;             // 选择 ADCSOC4 的输入为 ADCINB6
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC4由ePWM1的ADCSOCA事件触发
    AdcbRegs.ADCINTSOCSEL1.bit.SOC4 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC4

    // -----------------------------------------------
    // SYSCLK=100M  ADCCLK = 100M/4 = 25M
    // ADC采样和保持电路(S+H)由SYSCLK时钟，而ADC转换过程由ADCCLK时钟。
    // 采样窗口计算公式:
    // 采集时间 = (ACQPS + 1) × SYSCLK周期
    // 5个SYSCLK周期 = 5 × 10ns = 50ns
    // -----------------------------------------------

    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 4;     // SOC0采样窗口: 5个SYSCLK周期 (50ns)
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 4;     // SOC1采样窗口: 5个SYSCLK周期 (50ns)
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 4;     // SOC2采样窗口: 5个SYSCLK周期 (50ns)
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 4;     // SOC3采样窗口: 5个SYSCLK周期 (50ns)
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = 4;     // SOC4采样窗口: 5个SYSCLK周期 (50ns)
    // -----------------------------------------------

    // 突发模式配置
    // 注: 突发模式允许单个触发信号启动多个SOC转换
    AdcbRegs.ADCBURSTCTL.bit.BURSTEN = ADC_BURSTEN_DISABLE;       // 禁用突发模式
    //AdcaRegs.ADCBURSTCTL.bit.BURSTEN = ADC_BURSTEN_ENABLE;     // 使能  Burst Mode
    //AdcaRegs.ADCBURSTCTL.bit.BURSTTRIGSEL = ADC_BURSTTRIGSEL_ePWM7ADCSOCA; // 设置 Burst 的触发源

    // SOC优先级配置
    AdcbRegs.ADCSOCPRICTL.bit.RRPOINTER = ADCSOC_RRPOINTER_Reset; // 循环优先级指针复位到SOC0
    AdcbRegs.ADCSOCPRICTL.bit.SOCPRIORITY = ADCSOC_PRIORITY_RR;   // 所有通道使用循环优先级模式

    // ADC_B 中断配置
    // INT1 和 INT2
    // -----------------------------------------------
//    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = ADC_INTxSEL_EOC0;       // ADC_B 中断1 触发源 SOC0
//    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = ADC_INTxCONT_DISABLE;  // ADC_B 中断1 连续中断禁止
//    AdcbRegs.ADCINTSEL1N2.bit.INT1E = ADC_INTx_ENABLE;          // ADC_B 中断1 使能
//    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = ADC_ADCINTxFLG_CLEAR;   // ADC_B中断1 标志清零

    EDIS;
}


// =============================================================================
/*******************************************************************************
 函数名称：    void ConfigureADCC_SOC(void)
 功能描述：    a.配置ADCC模块的启动转换(SOC)参数，用于交流电压采集
              b.配置5个SOC全部使用ADCINC14通道，由ePWM1的ADCSOCA事件触发
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    a.使用ADCINC14通道(J7-65)进行交流电压采集
              b.配置5个SOC实现过采样，提高测量精度
              c.采样由ePWM1模块同步触发，确保采样时刻准确性
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、配置 ADC 参考源
 * ADC 参考源 1、外部参考源 2、内部参考源
 * 内部参考源 0 (0-3.3) 1(0-2.5)
 *
 * 2、配置 ADC 的时钟分频系数
 * SYSCLK = 100M
 * ADCCLK = 100M/4 = 25M
 *
 * 3、配置 ADC Interrupt Pulse Position （ ADC 中断脉冲产生的时机 ）
 * 0  ADC Interrupt Pulse begins conversion
 * 1  ADC Interrupt Pulse end of the conversion
 *
 * 4、 Power up the ADC （ ADC 模块上电 ）
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1557
 *----------------------------------------------------------*/
void ConfigureADCC_SOC(void){
    // -----------------------------------------------
    // 交流电压采样
    // 开发板硬件端口 ADCINC14 J7-65
    // 软件使用过采样 SOC0 - SOC4 共 5 个 SOC
    // -----------------------------------------------

    EALLOW;     // 允许访问受保护的寄存器

    // SOC0 配置
    // -----------------------------------------------
    AdccRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHSEL_ADCIN14;            // 选择 SOC0 的输入为 ADCINC14
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC0由ePWM1的ADCSOCA事件触发
    AdccRegs.ADCINTSOCSEL1.bit.SOC0 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC0

    // ADCINC1 配置
    // -----------------------------------------------
    AdccRegs.ADCSOC1CTL.bit.CHSEL = ADC_CHSEL_ADCIN14;            // 选择 SOC1 的输入为 ADCINC14
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC1由ePWM1的ADCSOCA事件触发
    AdccRegs.ADCINTSOCSEL1.bit.SOC1 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC1

    // ADCINC2 配置
    // -----------------------------------------------
    AdccRegs.ADCSOC2CTL.bit.CHSEL = ADC_CHSEL_ADCIN14;            // 选择 SOC2 的输入为 ADCINC14
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC2由ePWM1的ADCSOCA事件触发
    AdccRegs.ADCINTSOCSEL1.bit.SOC2 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC2

    // ADCINC3 配置
    // -----------------------------------------------
    AdccRegs.ADCSOC3CTL.bit.CHSEL = ADC_CHSEL_ADCIN14;            // 选择 SOC3 的输入为 ADCINC14
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC3由ePWM1的ADCSOCA事件触发
    AdccRegs.ADCINTSOCSEL1.bit.SOC3 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC3

    // ADCINC4 配置
    // -----------------------------------------------
    AdccRegs.ADCSOC4CTL.bit.CHSEL = ADC_CHSEL_ADCIN14;            // 选择 SOC4 的输入为 ADCINC14
    AdccRegs.ADCSOC4CTL.bit.TRIGSEL = ADC_TRIGSEL_ePWM1ADCSOCA;   // 设置SOC4由ePWM1的ADCSOCA事件触发
    AdccRegs.ADCINTSOCSEL1.bit.SOC4 = ADCINT_Trigger_DISABLE;     // 禁止ADC中断触发SOC4

    // -----------------------------------------------
    // SYSCLK=100M  ADCCLK = 100M/4 = 25M
    // ADC采样和保持电路(S+H)由SYSCLK时钟，而ADC转换过程由ADCCLK时钟。
    // 采样窗口计算公式:
    // 采集时间 = (ACQPS + 1) × SYSCLK周期
    // 5个SYSCLK周期 = 5 × 10ns = 50ns
    // -----------------------------------------------
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 4;     //SOC0 sample window is 5 SYSCLK cycles
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 4;     //SOC1 sample window is 5 SYSCLK cycles
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 4;     //SOC2 sample window is 5 SYSCLK cycles
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 4;     //SOC3 sample window is 5 SYSCLK cycles
    AdccRegs.ADCSOC4CTL.bit.ACQPS = 4;     //SOC4 sample window is 5 SYSCLK cycles
    // -----------------------------------------------

    // 突发模式配置
    // 注: 突发模式允许单个触发信号启动多个SOC转换
    AdccRegs.ADCBURSTCTL.bit.BURSTEN = ADC_BURSTEN_DISABLE;     // 禁用突发模式
    //AdccRegs.ADCBURSTCTL.bit.BURSTEN = ADC_BURSTEN_ENABLE;    // 使能  Burst Mode
    //AdccRegs.ADCBURSTCTL.bit.BURSTTRIGSEL = ADC_BURSTTRIGSEL_ePWM7ADCSOCA; // 设置 Burst 的触发源

    // 配置 SOCx 的优先级
    AdccRegs.ADCSOCPRICTL.bit.RRPOINTER = ADCSOC_RRPOINTER_Reset;     // 循环优先级指针复位到SOC0
    AdccRegs.ADCSOCPRICTL.bit.SOCPRIORITY = ADCSOC_PRIORITY_RR;       // 所有通道使用循环优先级模式

    // ADC_C 中断配置
    //    AdccRegs.ADCINTSEL1N2.bit.INT1CONT = ADC_INTxCONT_DISABLE;  // ADC_C 中断1 连续中断禁止
    //    AdccRegs.ADCINTSEL1N2.bit.INT1E = ADC_INTx_ENABLE;          // ADC_C 中断1 使能
    //    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = ADC_ADCINTxFLG_CLEAR;   // ADC_C中断1 标志清零
    EDIS;   // 禁止访问受保护的寄存器
}

/* *******************************************************
 *
 * 配置 ADC Post-Processing Blocks
 *
 * *******************************************************/
/* -----------------------------------------------------------------------
 * 每个 ADC 模块包含 4 个后处理块（PPB），有 4 种处理方式
 *
 * 1、 对原始的 ADC 值进行加减修正
 * 2、 flag a zero-crossing point
 * 3、 flag a high compare limit
 * 4、 flag a low compare limit
 *
 -----------------------------------------------------------------------*/

/*******************************************************************************
 函数名称：    void ConfigureADCA_PPB1(void)
 功能描述：    a.配置ADC-A模块的后处理块1(PPB1)参数
              b.PPB用于对ADC转换结果进行偏移校准、限值检查等后处理操作
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    a.配置PPB1处理SOC4的转换结果
              b.禁用所有PPB事件和中断功能
              c.可用于后续添加偏移校准或限值检查功能
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、 配置 PPB1 处理 SOCx 通道
 * 2、 配置 PPB1 处理 SOCx 源 是否使用 2' 的补码
 * 3、 配置 PPB1 的事件自动清除
 * 4、 配置 PPB1 的 PPBxOFFCAL 对原始的 ADC 值修正后并覆盖原值 ADCRESULT
 * 5、 配置 PPB1 的 PPBxOFFREF 对 ADC 数据进行修正并保存在 PPBxRESULT
 * 6、 配置 PPB1 的高低限制值
 * 7、 禁止所有事件和中断
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1557
 ------------------------------------------------------------*/
void ConfigureADCA_PPB1(void){

    EALLOW; // 解锁受保护的寄存器

    // PPB1基本配置
    AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = PPBxCONFIG_CONFIG_SOC4;        // 配置 PPB1 处理 SOC4 的转换结果
    AdcaRegs.ADCPPB1CONFIG.bit.TWOSCOMPEN = PPBxCONFIG_TWOSCOMPEN_POS; // 禁用二进制补码格式，使用原始正数格式
    AdcaRegs.ADCPPB1CONFIG.bit.CBCEN = PPBxCONFIG_CBCEN_DISABLE;       // 禁用硬件自动清除事件标志功能

    // 偏移校准配置
    // 校准公式: 最终值 = 原始值 + OFFCAL - OFFREF
    AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // 偏移校准值设置为0 (范围: -1 到 +512)
    AdcaRegs.ADCPPB1OFFREF = 0;  // 偏移参考值设置为0 (12位范围: -1 到 -4096)

    // 限值检查配置
    AdcaRegs.ADCPPB1TRIPHI.bit.LIMITHI = 0;     // 设置高限值阈值为0 (禁用高限值检查)
    AdcaRegs.ADCPPB1TRIPLO.bit.LIMITLO = 0;    // 设置低限值阈值为0 (禁用低限值检查)

    // PPB事件配置 - 全部禁用
    AdcaRegs.ADCEVTSEL.bit.PPB1TRIPHI = PPBxTRIPHI_EventDISABLE; // 禁用超过高限值事件
    AdcaRegs.ADCEVTSEL.bit.PPB1TRIPLO = PPBxTRIPLO_EventDISABLE; // 禁用低于低限值事件
    AdcaRegs.ADCEVTSEL.bit.PPB1ZERO = PPBxZERO_EventDISABLE;     // 禁用零值事件

    // PPB中断配置 - 全部禁用
    // 使能事件
//    AdcaRegs.ADCEVTSEL.bit.PPB1TRIPHI = PPBxTRIPHI_EventENABLE; // 使能PPB1限值触发事件
//    AdcaRegs.ADCEVTSEL.bit.PPB1TRIPLO = PPBxTRIPLO_EventENABLE; // 使能PPB1低限值触发事件
//    AdcaRegs.ADCEVTSEL.bit.PPB1ZERO = PPBxZERO_EventENABLE;     // 使能PPB1零值触发事件

    // 清除 PPB 事件标志
    AdcaRegs.ADCEVTCLR.bit.PPB1TRIPHI = PPBxTRIPHI_EventClear;      // 清除高限值事件标志
    AdcaRegs.ADCEVTCLR.bit.PPB1TRIPLO = PPBxTRIPLO_EventClear;      // 清除低限值事件标志
    AdcaRegs.ADCEVTCLR.bit.PPB1ZERO = PPBxZERO_EventClear;          // 清除零值事件标志

    // 配置 PPB 中断
//    AdcaRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = PPBxTRIPHI_INTENABLE;  // 使能高限值中断
//    AdcaRegs.ADCEVTINTSEL.bit.PPB1TRIPLO = PPBxTRIPLO_INTENABLE;  // 使能低限值中断
//    AdcaRegs.ADCEVTINTSEL.bit.PPB1ZERO = PPBxZERO_INTENABLE;      // 使能零值中断

    // PPB中断配置 - 全部禁用
    AdcaRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = PPBxTRIPHI_INTDISABLE;   // 禁用高限值中断
    AdcaRegs.ADCEVTINTSEL.bit.PPB1TRIPLO = PPBxTRIPLO_INTDISABLE;   // 禁用低限值中断
    AdcaRegs.ADCEVTINTSEL.bit.PPB1ZERO = PPBxZERO_INTDISABLE;       // 禁用零值中断

    EDIS; // 重新锁定受保护的寄存器
}

/*******************************************************************************
 函数名称：    void ConfigureADCA_PPB2(void)
 功能描述：    a.配置ADC-A模块的后处理块2(PPB2)参数
              b.PPB用于对ADC转换结果进行偏移校准、限值检查等后处理操作
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    a.配置PPB2处理SOC0的转换结果
              b.禁用所有PPB事件和中断功能
              c.可用于后续添加偏移校准或限值检查功能
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、 配置 PPB2 处理 SOCx 通道
 * 2、 配置 PPB2 处理 SOCx 源 是否使用 2' 的补码
 * 3、 配置 PPB2 的事件自动清除
 * 4、 配置 PPB2 的 PPBxOFFCAL 对原始的 ADC 值修正后并覆盖原值 ADCRESULT
 * 5、 配置 PPB2 的 PPBxOFFREF 对 ADC 数据进行修正并保存在 PPBxRESULT
 * 6、 配置 PPB2 的高低限制值
 * 7、 禁止所有事件和中断
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1557
 ------------------------------------------------------------*/
//void ConfigureADCA_PPB2(void){
//
//    EALLOW; // 解锁受保护的寄存器
//
//    // PPB2基本配置
//    AdcaRegs.ADCPPB2CONFIG.bit.CONFIG = PPBxCONFIG_CONFIG_SOC0;        // 配置 PPB2处理 SOC0 的转换结果
//    AdcaRegs.ADCPPB2CONFIG.bit.TWOSCOMPEN = PPBxCONFIG_TWOSCOMPEN_POS; // 禁用二进制补码格式，使用原始正数格式
//    AdcaRegs.ADCPPB2CONFIG.bit.CBCEN = PPBxCONFIG_CBCEN_DISABLE;       // 禁用硬件自动清除事件标志功能
//
//    // 偏移校准配置
//    // 校准公式: 最终值 = 原始值 + OFFCAL - OFFREF
//    AdcaRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;    // 偏移校准值设置为0 (范围: -1 到 +512)
//    AdcaRegs.ADCPPB2OFFREF = 0;  // 偏移参考值设置为0 (12位范围: -1 到 -4096)
//
//    // 限值检查配置
//    AdcaRegs.ADCPPB2TRIPHI.bit.LIMITHI = 0;     // 设置高限值阈值为0 (禁用高限值检查)
//    AdcaRegs.ADCPPB2TRIPLO.bit.LIMITLO = 0;    // 设置低限值阈值为0 (禁用低限值检查)
//
//    // PPB事件配置 - 全部禁用
//    AdcaRegs.ADCEVTSEL.bit.PPB2TRIPHI = PPBxTRIPHI_EventDISABLE; // 禁用超过高限值事件
//    AdcaRegs.ADCEVTSEL.bit.PPB2TRIPLO = PPBxTRIPLO_EventDISABLE; // 禁用低于低限值事件
//    AdcaRegs.ADCEVTSEL.bit.PPB2ZERO = PPBxZERO_EventDISABLE;     // 禁用零值事件
//
//    // PPB中断配置 - 全部禁用
//    // 使能事件
////    AdcaRegs.ADCEVTSEL.bit.PPB2TRIPHI = PPBxTRIPHI_EventENABLE; // 使能PPB2限值触发事件
////    AdcaRegs.ADCEVTSEL.bit.PPB2TRIPLO = PPBxTRIPLO_EventENABLE; // 使能PPB2低限值触发事件
////    AdcaRegs.ADCEVTSEL.bit.PPB2ZERO = PPBxZERO_EventENABLE;     // 使能PPB2零值触发事件
//
//    // 清除 PPB 事件标志
//    AdcaRegs.ADCEVTCLR.bit.PPB2TRIPHI = PPBxTRIPHI_EventClear;      // 清除高限值事件标志
//    AdcaRegs.ADCEVTCLR.bit.PPB2TRIPLO = PPBxTRIPLO_EventClear;      // 清除低限值事件标志
//    AdcaRegs.ADCEVTCLR.bit.PPB2ZERO = PPBxZERO_EventClear;          // 清除零值事件标志
//
//    // 配置 PPB 中断
////    AdcaRegs.ADCEVTINTSEL.bit.PPB2TRIPHI = PPBxTRIPHI_INTENABLE;  // 使能高限值中断
////    AdcaRegs.ADCEVTINTSEL.bit.PPB2TRIPLO = PPBxTRIPLO_INTENABLE;  // 使能低限值中断
////    AdcaRegs.ADCEVTINTSEL.bit.PPB2ZERO = PPBxZERO_INTENABLE;      // 使能零值中断
//
//    // PPB中断配置 - 全部禁用
//    AdcaRegs.ADCEVTINTSEL.bit.PPB2TRIPHI = PPBxTRIPHI_INTDISABLE;   // 禁用高限值中断
//    AdcaRegs.ADCEVTINTSEL.bit.PPB2TRIPLO = PPBxTRIPLO_INTDISABLE;   // 禁用低限值中断
//    AdcaRegs.ADCEVTINTSEL.bit.PPB2ZERO = PPBxZERO_INTDISABLE;       // 禁用零值中断
//
//    EDIS; // 重新锁定受保护的寄存器
//}

/*******************************************************************************
 函数名称：    void ConfigureADCA_PPB3(void)
 功能描述：    a.配置ADC-A模块的后处理块3(PPB3)参数
              b.PPB用于对ADC转换结果进行偏移校准、限值检查等后处理操作
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    a.配置PPB3处理SOC0的转换结果
              b.禁用所有PPB事件和中断功能
              c.可用于后续添加偏移校准或限值检查功能
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、 配置 PPB3 处理 SOCx 通道
 * 2、 配置 PPB3 处理 SOCx 源 是否使用 2' 的补码
 * 3、 配置 PPB3 的事件自动清除
 * 4、 配置 PPB3 的 PPBxOFFCAL 对原始的 ADC 值修正后并覆盖原值 ADCRESULT
 * 5、 配置 PPB3 的 PPBxOFFREF 对 ADC 数据进行修正并保存在 PPBxRESULT
 * 6、 配置 PPB3 的高低限制值
 * 7、 禁止所有事件和中断
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1557
 ------------------------------------------------------------*/
//void ConfigureADCA_PPB3(void){
//
//    EALLOW; // 解锁受保护的寄存器
//
//    // PPB3基本配置
//    AdcaRegs.ADCPPB3CONFIG.bit.CONFIG = PPBxCONFIG_CONFIG_SOC0;        // 配置 PPB3 处理 SOC0 的转换结果
//    AdcaRegs.ADCPPB3CONFIG.bit.TWOSCOMPEN = PPBxCONFIG_TWOSCOMPEN_POS; // 禁用二进制补码格式，使用原始正数格式
//    AdcaRegs.ADCPPB3CONFIG.bit.CBCEN = PPBxCONFIG_CBCEN_DISABLE;       // 禁用硬件自动清除事件标志功能
//
//    // 偏移校准配置
//    // 校准公式: 最终值 = 原始值 + OFFCAL - OFFREF
//    AdcaRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // 偏移校准值设置为0 (范围: -1 到 +512)
//    AdcaRegs.ADCPPB3OFFREF = 0;  // 偏移参考值设置为0 (12位范围: -1 到 -4096)
//
//    // 限值检查配置
//    AdcaRegs.ADCPPB3TRIPHI.bit.LIMITHI = 0;     // 设置高限值阈值为0 (禁用高限值检查)
//    AdcaRegs.ADCPPB3TRIPLO.bit.LIMITLO = 0;    // 设置低限值阈值为0 (禁用低限值检查)
//
//    // PPB事件配置 - 全部禁用
//    AdcaRegs.ADCEVTSEL.bit.PPB3TRIPHI = PPBxTRIPHI_EventDISABLE; // 禁用超过高限值事件
//    AdcaRegs.ADCEVTSEL.bit.PPB3TRIPLO = PPBxTRIPLO_EventDISABLE; // 禁用低于低限值事件
//    AdcaRegs.ADCEVTSEL.bit.PPB3ZERO = PPBxZERO_EventDISABLE;     // 禁用零值事件
//
//    // PPB中断配置 - 全部禁用
//    // 使能事件
////    AdcaRegs.ADCEVTSEL.bit.PPB3TRIPHI = PPBxTRIPHI_EventENABLE; // 使能PPB3限值触发事件
////    AdcaRegs.ADCEVTSEL.bit.PPB3TRIPLO = PPBxTRIPLO_EventENABLE; // 使能PPB3低限值触发事件
////    AdcaRegs.ADCEVTSEL.bit.PPB3ZERO = PPBxZERO_EventENABLE;     // 使能PPB3零值触发事件
//
//    // 清除 PPB 事件标志
//    AdcaRegs.ADCEVTCLR.bit.PPB3TRIPHI = PPBxTRIPHI_EventClear;      // 清除高限值事件标志
//    AdcaRegs.ADCEVTCLR.bit.PPB3TRIPLO = PPBxTRIPLO_EventClear;      // 清除低限值事件标志
//    AdcaRegs.ADCEVTCLR.bit.PPB3ZERO = PPBxZERO_EventClear;          // 清除零值事件标志
//
//    // 配置 PPB 中断
////    AdcaRegs.ADCEVTINTSEL.bit.PPB3TRIPHI = PPBxTRIPHI_INTENABLE;  // 使能高限值中断
////    AdcaRegs.ADCEVTINTSEL.bit.PPB3TRIPLO = PPBxTRIPLO_INTENABLE;  // 使能低限值中断
////    AdcaRegs.ADCEVTINTSEL.bit.PPB3ZERO = PPBxZERO_INTENABLE;      // 使能零值中断
//
//    // PPB中断配置 - 全部禁用
//    AdcaRegs.ADCEVTINTSEL.bit.PPB3TRIPHI = PPBxTRIPHI_INTDISABLE;   // 禁用高限值中断
//    AdcaRegs.ADCEVTINTSEL.bit.PPB3TRIPLO = PPBxTRIPLO_INTDISABLE;   // 禁用低限值中断
//    AdcaRegs.ADCEVTINTSEL.bit.PPB3ZERO = PPBxZERO_INTDISABLE;       // 禁用零值中断
//
//    EDIS; // 重新锁定受保护的寄存器
//}


/*******************************************************************************
 函数名称：    void ConfigureADCA_PPB4(void)
 功能描述：    a.配置ADC-A模块的后处理块4(PPB4)参数
              b.PPB用于对ADC转换结果进行偏移校准、限值检查等后处理操作
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    a.配置PPB4处理SOC0的转换结果
              b.禁用所有PPB事件和中断功能
              c.可用于后续添加偏移校准或限值检查功能
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、 配置 PPB4 处理 SOCx 通道
 * 2、 配置 PPB4 处理 SOCx 源 是否使用 2' 的补码
 * 3、 配置 PPB4 的事件自动清除
 * 4、 配置 PPB4 的 PPBxOFFCAL 对原始的 ADC 值修正后并覆盖原值 ADCRESULT
 * 5、 配置 PPB4 的 PPBxOFFREF 对 ADC 数据进行修正并保存在 PPBxRESULT
 * 6、 配置 PPB4 的高低限制值
 * 7、 禁止所有事件和中断
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1557
 ------------------------------------------------------------*/
//void ConfigureADCA_PPB4(void){
//
//    EALLOW; // 解锁受保护的寄存器
//
//    // PPB4基本配置
//    AdcaRegs.ADCPPB4CONFIG.bit.CONFIG = PPBxCONFIG_CONFIG_SOC0;        // 配置 PPB4 处理 SOC0 的转换结果
//    AdcaRegs.ADCPPB4CONFIG.bit.TWOSCOMPEN = PPBxCONFIG_TWOSCOMPEN_POS; // 禁用二进制补码格式，使用原始正数格式
//    AdcaRegs.ADCPPB4CONFIG.bit.CBCEN = PPBxCONFIG_CBCEN_DISABLE;       // 禁用硬件自动清除事件标志功能
//
//    // 偏移校准配置
//    // 校准公式: 最终值 = 原始值 + OFFCAL - OFFREF
//    AdcaRegs.ADCPPB4OFFCAL.bit.OFFCAL = 0;    // 偏移校准值设置为0 (范围: -1 到 +512)
//    AdcaRegs.ADCPPB4OFFREF = 0;  // 偏移参考值设置为0 (12位范围: -1 到 -4096)
//
//    // 限值检查配置
//    AdcaRegs.ADCPPB4TRIPHI.bit.LIMITHI = 0;     // 设置高限值阈值为0 (禁用高限值检查)
//    AdcaRegs.ADCPPB4TRIPLO.bit.LIMITLO = 0;    // 设置低限值阈值为0 (禁用低限值检查)
//
//    // PPB事件配置 - 全部禁用
//    AdcaRegs.ADCEVTSEL.bit.PPB4TRIPHI = PPBxTRIPHI_EventDISABLE; // 禁用超过高限值事件
//    AdcaRegs.ADCEVTSEL.bit.PPB4TRIPLO = PPBxTRIPLO_EventDISABLE; // 禁用低于低限值事件
//    AdcaRegs.ADCEVTSEL.bit.PPB4ZERO = PPBxZERO_EventDISABLE;     // 禁用零值事件
//
//    // PPB中断配置 - 全部禁用
//    // 使能事件
////    AdcaRegs.ADCEVTSEL.bit.PPB4TRIPHI = PPBxTRIPHI_EventENABLE; // 使能PPB4限值触发事件
////    AdcaRegs.ADCEVTSEL.bit.PPB4TRIPLO = PPBxTRIPLO_EventENABLE; // 使能PPB4低限值触发事件
////    AdcaRegs.ADCEVTSEL.bit.PPB4ZERO = PPBxZERO_EventENABLE;     // 使能PPB4零值触发事件
//
//    // 清除 PPB 事件标志
//    AdcaRegs.ADCEVTCLR.bit.PPB4TRIPHI = PPBxTRIPHI_EventClear;      // 清除高限值事件标志
//    AdcaRegs.ADCEVTCLR.bit.PPB4TRIPLO = PPBxTRIPLO_EventClear;      // 清除低限值事件标志
//    AdcaRegs.ADCEVTCLR.bit.PPB4ZERO = PPBxZERO_EventClear;          // 清除零值事件标志
//
//    // 配置 PPB 中断
////    AdcaRegs.ADCEVTINTSEL.bit.PPB4TRIPHI = PPBxTRIPHI_INTENABLE;  // 使能高限值中断
////    AdcaRegs.ADCEVTINTSEL.bit.PPB4TRIPLO = PPBxTRIPLO_INTENABLE;  // 使能低限值中断
////    AdcaRegs.ADCEVTINTSEL.bit.PPB4ZERO = PPBxZERO_INTENABLE;      // 使能零值中断
//
//    // PPB中断配置 - 全部禁用
//    AdcaRegs.ADCEVTINTSEL.bit.PPB4TRIPHI = PPBxTRIPHI_INTDISABLE;   // 禁用高限值中断
//    AdcaRegs.ADCEVTINTSEL.bit.PPB4TRIPLO = PPBxTRIPLO_INTDISABLE;   // 禁用低限值中断
//    AdcaRegs.ADCEVTINTSEL.bit.PPB4ZERO = PPBxZERO_INTDISABLE;       // 禁用零值中断
//
//    EDIS; // 重新锁定受保护的寄存器
//}


