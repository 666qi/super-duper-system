//###########################################################################
//
// FILE:    f28004x_cputimers.c
//
// TITLE:   f28004x CPU 32-bit Timers Initialization & Support Functions.
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
#include "f28004x_device.h"      // Headerfile Include File
#include "f28004x_examples.h"    // Examples Include File

//
// Globals
//
struct CPUTIMER_VARS CpuTimer0;
struct CPUTIMER_VARS CpuTimer1;
struct CPUTIMER_VARS CpuTimer2;




/*******************************************************************************
 函数名称：    void InitCpuTimers(void)
 功能描述：    初始化三个CPU定时器（Timer0/1/2），配置基础参数并停止定时器
 输入参数：    无
 输出参数：    无
 返 回 值：   无
 其它说明：   a. 设置定时器周期为最大值（0xFFFFFFFF）
           b. 预分频器配置为1分频（SYSCLKOUT直接驱动）
           c. 强制重载计数器初始值
           d. 初始化定时器中断计数器
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/10/27    V1.0           LW             创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1. 关联定时器寄存器地址指针
 * 2. 设置定时器周期寄存器（PRD）为最大值
 * 3. 清零预分频寄存器（TPR/TPRH）实现1分频
 * 4. 停止定时器运行（TCR.TSS=1）
 * 5. 强制重载计数器（TCR.TRB=1）
 * 6. 复位中断计数变量
 * ----------------------------------------------------------*/
void InitCpuTimers(void)
{

    // ====================== CPU Timer 0 配置 ======================
    // 将定时器0的寄存器地址指针指向实际硬件寄存器
    CpuTimer0.RegsAddr = &CpuTimer0Regs;

    // 设置定时器0周期寄存器为最大值（0xFFFFFFFF）
    // 相当于定时器从最大值开始递减计数
    CpuTimer0Regs.PRD.all  = 0xFFFFFFFF;

    // 配置预分频寄存器（低8位和高8位）
    // TPR = 0 表示1分频（不分频），TPRH = 0 保留
    CpuTimer0Regs.TPR.all  = 0;         // 低8位预分频值
    CpuTimer0Regs.TPRH.all = 0;         // 高8位预分频值（保留位）

    // 停止定时器0运行（TSS=1 表示停止定时器）
    CpuTimer0Regs.TCR.bit.TSS = 1;

    // 强制重载计数器（TRB=1 将周期值PRD加载到计数器）
    CpuTimer0Regs.TCR.bit.TRB = 1;


    // 初始化定时器0中断计数器为0
    CpuTimer0.InterruptCount = 0;

    // ====================== CPU Timer 1 & 2 配置 ======================
    // 将定时器1和2的寄存器地址指针指向实际硬件寄存器
    CpuTimer1.RegsAddr = &CpuTimer1Regs;
    CpuTimer2.RegsAddr = &CpuTimer2Regs;

    // 设置定时器1和2周期寄存器为最大值（0xFFFFFFFF）
    CpuTimer1Regs.PRD.all  = 0xFFFFFFFF;
    CpuTimer2Regs.PRD.all  = 0xFFFFFFFF;

    // 配置预分频寄存器（1分频配置）
    CpuTimer1Regs.TPR.all  = 0;  // 定时器1低8位预分频
    CpuTimer1Regs.TPRH.all = 0;  // 定时器1高8位预分频
    CpuTimer2Regs.TPR.all  = 0;  // 定时器2低8位预分频
    CpuTimer2Regs.TPRH.all = 0;  // 定时器2高8位预分频

    // 停止定时器1和2运行（防止初始化过程中意外启动）
    CpuTimer1Regs.TCR.bit.TSS = 1;
    CpuTimer2Regs.TCR.bit.TSS = 1;

    // 强制重载定时器1和2的计数器
    CpuTimer1Regs.TCR.bit.TRB = 1;
    CpuTimer2Regs.TCR.bit.TRB = 1;

    // 初始化定时器1和2的中断计数器为0
    CpuTimer1.InterruptCount = 0;
    CpuTimer2.InterruptCount = 0;
}


/*******************************************************************************
 函数名称：    void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period)
 功能描述：    配置指定CPU定时器的周期和运行参数
 输入参数：    Timer   - 指向定时器配置结构的指针
            Freq    - CPU时钟频率（单位：MHz）
            Period  - 定时器周期（单位：微秒）
 输出参数：    无
 返 回 值：   无
 其它说明：   a. 计算并设置定时器周期值
           b. 配置预分频器为1分频（SYSCLKOUT直接驱动）
           c. 配置定时器控制寄存器（TCR）参数：
                 - 停止定时器（TSS=1）
                 - 强制重载计数器（TRB=1）
                 - 禁用软停止模式（SOFT=0）
                 - 禁用自由运行模式（FREE=0）
                 - 使能定时器中断（TIE=1）
           d. 复位中断计数器
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/10/27    V1.0           LW             创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1. 计算定时器周期值（Freq * Period）
 * 2. 设置周期寄存器（PRD）
 * 3. 配置预分频寄存器（TPR/TPRH）为1分频
 * 4. 配置定时器控制寄存器（TCR）：
 *    a. 停止定时器（TSS=1）
 *    b. 强制重载计数器（TRB=1）
 *    c. 设置SOFT和FREE模式
 *    d. 使能中断（TIE=1）
 * 5. 复位中断计数器
 *----------------------------------------------------------*/
void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period)
{
    Uint32 temp;   // 临时变量用于计算周期值

    // ====================== 定时器周期配置 ======================
    // 保存CPU频率（MHz）和定时周期（微秒）到定时器结构体
    Timer->CPUFreqInMHz = Freq;
    Timer->PeriodInUSec = Period;

    // 计算定时器周期值：频率(MHz) * 周期(us) = 定时器计数次数
    temp = (long) (Freq * Period);

    // 将计算出的周期值写入定时器的周期寄存器（PRD）
    Timer->RegsAddr->PRD.all = temp;

    // ====================== 预分频器配置 ======================
    // 设置预分频寄存器低8位为0（1分频）
    Timer->RegsAddr->TPR.all  = 0;

    // 设置预分频寄存器高8位为0（保留位）
    Timer->RegsAddr->TPRH.all  = 0;

    // ====================== 定时器控制寄存器配置 ======================
    // 停止定时器（TSS=1：定时器停止状态）
    Timer->RegsAddr->TCR.bit.TSS = 1;

    // 强制重载计数器（TRB=1：将PRD值加载到计数器）
    Timer->RegsAddr->TCR.bit.TRB = 1;

    // 禁用软停止模式（SOFT=0：定时器在仿真停止时立即停止）
    Timer->RegsAddr->TCR.bit.SOFT = 0;

    // 禁用自由运行模式（FREE=0：定时器受仿真事件影响）
    Timer->RegsAddr->TCR.bit.FREE = 1;

    // 使能定时器中断（TIE=1：定时器中断使能）
    Timer->RegsAddr->TCR.bit.TIE = 1;

    // ====================== 中断计数器初始化 ======================
    // 复位中断计数变量（记录定时器中断发生次数）
    Timer->InterruptCount = 0;
}


//
// End of File
//

