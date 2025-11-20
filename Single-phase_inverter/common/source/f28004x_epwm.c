/*
 * f28004x_epwm.c
 *
 *  Created on: 2025年8月27日
 *      Author: jzy
 */

//
// 头文件
//
#include "f28004x_device.h"
#include "f28004x_examples.h"
#include "configs.h"
#include "board.h"


// 自己添加的函数
/* --------------------------------------------------------------
 * 用户根据自己的实际需求修改
 * --------------------------------------------------------------*/


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ePWM1GPIO(void)
 功能描述：    配置 ePWM1 模块对应的 GPIO 引脚功能
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    此函数配置 GPIO0 和 GPIO1 作为 ePWM1 输出引脚
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW             创建
********************************************************************************/
/*----------------------------------------------------------
 * 函数功能:配置 ePWM1 模块的 GPIO 引脚功能
 * ----------------------------------------------------------
 * 配置步骤：
 * 1. 禁用内部上拉电阻 (GPxPUD)
 * 2. 配置多路复用器选择外设功能 (GPxGMUX, GPxMUX)
 *----------------------------------------------------------*/
void Init_ePWM1GPIO(void){

    EALLOW;   // 允许访问受保护的寄存器

    // 禁用内部上拉电阻（PWM输出通常不需要上拉）
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = PUD_DISABLE;   // 禁用 GPIO0 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = PUD_DISABLE;   // 禁用 GPIO1 上拉电阻

    // 配置 GPIO0 为 ePWM1A 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;  // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // 配置为外设功能 1 (ePWM1A)

    // 配置 GPIO1 为 ePWM1B 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;  // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // 配置为外设功能 1 (ePWM1B)

    EDIS;   // 禁止访问受保护的寄存器
}


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ePWM2GPIO(void)
 功能描述：    配置 ePWM2 模块对应的 GPIO 引脚功能
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    此函数配置 GPIO2 和 GPIO3 作为 ePWM2 输出引脚
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW             创建
********************************************************************************/
/*----------------------------------------------------------
 * 函数功能:配置 ePWM2模块的 GPIO 引脚功能
 * ----------------------------------------------------------
 * 配置步骤：
 * 1. 禁用内部上拉电阻 (GPxPUD)
 * 2. 配置多路复用器选择外设功能 (GPxGMUX, GPxMUX)
 *----------------------------------------------------------*/
void Init_ePWM2GPIO(void){

    EALLOW;   // 允许访问受保护的寄存器

    // 禁用内部上拉电阻（PWM输出通常不需要上拉）
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = PUD_DISABLE;   // 禁用 GPIO2 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = PUD_DISABLE;   // 禁用 GPIO3 上拉电阻

    // 配置 GPIO2 为 ePWM2A 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;           // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;           // 配置为外设功能 1 (ePWM2A)

    // 配置 GPIO3 为 ePWM2B 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;          // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;           // 配置为外设功能 1 (ePWM2B)

    EDIS;    // 禁止访问受保护的寄存器
}


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ePWM3GPIO(void)
 功能描述：    配置 ePWM3 模块对应的 GPIO 引脚功能
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    此函数配置 GPIO4 和 GPIO5 作为 ePWM3 输出引脚
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW             创建
********************************************************************************/
/*----------------------------------------------------------
 * 函数功能:配置 ePWM3模块的 GPIO 引脚功能
 * ----------------------------------------------------------
 * 配置步骤：
 * 1. 禁用内部上拉电阻 (GPxPUD)
 * 2. 配置多路复用器选择外设功能 (GPxGMUX, GPxMUX)
 *----------------------------------------------------------*/
void Init_ePWM3GPIO(void){

    EALLOW;  // 允许访问受保护的寄存器

    // 禁用内部上拉电阻（PWM输出通常不需要上拉）
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = PUD_ENABLE;   // 禁用 GPIO4 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = PUD_ENABLE;   // 禁用 GPIO5 上拉电阻

    // 配置 GPIO4 为 ePWM3A 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;          // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;          // 配置为外设功能 1 (ePWM3A)

    // 配置 GPIO5 为 ePWM3A 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;        // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;         // 配置为外设功能 1 (ePWM3B)

    EDIS;    // 禁止访问受保护的寄存器

}


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ePWM4GPIO(void)
 功能描述：    配置 ePWM4 模块对应的 GPIO 引脚功能
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    此函数配置 GPIO6 和 GPIO7 作为 ePWM4 输出引脚
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW             创建
********************************************************************************/
/*----------------------------------------------------------
 * 函数功能:配置 ePWM4模块的 GPIO 引脚功能
 * ----------------------------------------------------------
 * 配置步骤：
 * 1. 禁用内部上拉电阻 (GPxPUD)
 * 2. 配置多路复用器选择外设功能 (GPxGMUX, GPxMUX)
 *----------------------------------------------------------*/
void Init_ePWM4GPIO(void){

    EALLOW;   // 允许访问受保护的寄存器

    // 禁用内部上拉电阻（PWM输出通常不需要上拉）
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = PUD_ENABLE;   // 禁用 GPIO6 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = PUD_ENABLE;   // 禁用 GPIO7 上拉电阻

    // 配置 GPIO6 为 ePWM4A 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;// 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1; // 配置为外设功能 1 (ePWM4A)

    // 配置 GPIO7 为 ePWM4B 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;// 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;  // 配置为外设功能 1 (ePWM4B)

    EDIS;    // 禁止访问受保护的寄存器

}


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ePWM5GPIO(void)
 功能描述：    配置 ePWM5 模块对应的 GPIO 引脚功能
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    此函数配置 GPIO8 和 GPIO9 作为 ePWM5 输出引脚
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW             创建
********************************************************************************/
/*----------------------------------------------------------
 * 函数功能:配置 ePWM5模块的 GPIO 引脚功能
 * ----------------------------------------------------------
 * 配置步骤：
 * 1. 禁用内部上拉电阻 (GPxPUD)
 * 2. 配置多路复用器选择外设功能 (GPxGMUX, GPxMUX)
 *----------------------------------------------------------*/
void Init_ePWM5GPIO(void){

    EALLOW;   // 允许访问受保护的寄存器

    // 禁用内部上拉电阻（PWM输出通常不需要上拉）
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = PUD_ENABLE;   // 禁用 GPIO8 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = PUD_ENABLE;   // 禁用 GPIO9 上拉电阻

    // 配置 GPIO2 为 ePWM5A 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;         // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;         // 配置为外设功能 1 (ePWM5A)

    // 配置 GPIO2 为 ePWM5B 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;        // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;         // 配置为外设功能 1 (ePWM5B)

    EDIS;    // 禁止访问受保护的寄存器
}


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ePWM6GPIO(void)
 功能描述：    配置 ePWM6 模块对应的 GPIO 引脚功能
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    此函数配置 GPIO10 和 GPIO11 作为 ePWM6 输出引脚
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW             创建
********************************************************************************/
/*----------------------------------------------------------
 * 函数功能:配置 ePWM6模块的 GPIO 引脚功能
 * ----------------------------------------------------------
 * 配置步骤：
 * 1. 禁用内部上拉电阻 (GPxPUD)
 * 2. 配置多路复用器选择外设功能 (GPxGMUX, GPxMUX)
 *----------------------------------------------------------*/
void Init_ePWM6GPIO(void){

    EALLOW;   // 允许访问受保护的寄存器

    // 禁用内部上拉电阻（PWM输出通常不需要上拉）
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = PUD_ENABLE;   // 禁用 GPIO10 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = PUD_ENABLE;  // 禁用 GPIO11 上拉电阻

    // 配置 GPIO10 为 ePWM6A 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;        // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;         // 配置为外设功能 1 (ePWM6A)

    // 配置 GPIO11 为 ePWM6B 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;        // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;         // 配置为外设功能 1 (ePWM6B)

    EDIS;    // 禁止访问受保护的寄存器
}


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ePWM7GPIO(void)
 功能描述：    配置 ePWM7 模块对应的 GPIO 引脚功能
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    此函数配置 GPIO12 和 GPIO13 作为 ePWM7 输出引脚
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW             创建
********************************************************************************/
/*----------------------------------------------------------
 * 函数功能:配置 ePWM7模块的 GPIO 引脚功能
 * ----------------------------------------------------------
 * 配置步骤：
 * 1. 禁用内部上拉电阻 (GPxPUD)
 * 2. 配置多路复用器选择外设功能 (GPxGMUX, GPxMUX)
 *----------------------------------------------------------*/
void Init_ePWM7GPIO(void){

    EALLOW;   // 允许访问受保护的寄存器

    // 禁用内部上拉电阻（PWM输出通常不需要上拉）
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = PUD_ENABLE;   // 禁用 GPIO12 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = PUD_ENABLE;   // 禁用 GPIO13 上拉电阻

    // 配置 GPIO12 为 ePWM7A 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;         // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;          // 配置为外设功能 1 (ePWM7A)

    // 配置 GPIO13 为 ePWM7B 外设功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 0;         // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;         // 配置为外设功能 1 (ePWM7B)

    EDIS;    // 禁止访问受保护的寄存器
}

// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ePWM8GPIO(void)
 功能描述：    配置 ePWM8 模块对应的 GPIO 引脚功能
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    此函数配置 GPIO14 和 GPIO15 作为 ePWM8 输出引脚
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW             创建
********************************************************************************/
/*----------------------------------------------------------
 * 函数功能:配置 ePWM8模块的 GPIO 引脚功能
 * ----------------------------------------------------------
 * 配置步骤：
 * 1. 禁用内部上拉电阻 (GPxPUD)
 * 2. 配置多路复用器选择外设功能 (GPxGMUX, GPxMUX)
 *----------------------------------------------------------*/
void Init_ePWM8GPIO(void){

    EALLOW; // 允许访问受保护的寄存器

    // 禁用内部上拉电阻（PWM输出通常不需要上拉）
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = PUD_ENABLE;   // 禁用 GPIO14 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = PUD_ENABLE;   // 禁用 GPIO15 上拉电阻

    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;         // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;         // 配置为外设功能 1 (ePWM8A)

    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 0;       // 选择功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;        // 配置为外设功能 1 (ePWM8B)

    EDIS; // 禁止访问受保护的寄存器
}



// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ePWM1(void)
 功能描述：    配置 ePWM1 并用来产生一个 EPWMx_INT 中断 20KHz
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：   a.配置为上下计数模式，产生对称PWM波形
             b.启用死区生成，防止桥式电路直通
             c.配置事件触发用于ADC启动和中断生成
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW             创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、配置前停止 ePWM 外设时钟
 * 2、配置 Time-Base (TB) Submodule
 * 3、配置 Counter-Compare (CC) Submodule
 * 4、配置 Action-Qualifier (AQ) Submodule
 * 5、配置 Dead-Band Generator (DB) Submodule
 * 6、配置 Event-Trigger (ET) Submodule
 * 7、配置Digital Compare (DC) Submodule
 * 8、配置Trip-Zone (TZ) Submodule
 * ----------------------------------------------------------
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1877
 *----------------------------------------------------------*/
void Init_ePWM1(void){


    EALLOW;    // 允许访问受保护的寄存器

    // 1、配置前停止 ePWM 外设时钟
    //--------------------------------------------------------------------------------------
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;     // 停止所有 ePWM 时基时钟同步，确保安全配置

    // 2、配置 Time-Base (TB) Submodule
    //--------------------------------------------------------------------------------------
    /*
     * 时钟计算:
     * SYSCLK = EPWMCLK = 100M
     * TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)
     * TBCLK = 100M = 100M/1*1=100M
     *
     *TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1890
     * PWM频率计算 (上下计数模式):
     * PWM频率 = TBCLK / (2 × TBPRD)
     * 20kHz = 100MHz / (2 × TBPRD)
     * 因此 TBPRD = 2500
     */
    EPwm1Regs.TBCTL.bit.FREE_SOFT = TB_FREE;               // 仿真模式：自由运行，仿真挂起时PWM继续运行
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;         // 上下计数模式
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;                 // 周期寄存器装载模式：从影子寄存器装载
    EPwm1Regs.TBCTL.bit.PHSEN = TB_PHSEN_DISABLE;          // 相位装载使能：禁用相位装载
    EPwm1Regs.TBCTL.bit.PHSDIR = TB_PHSDIR_DOWN;           // 相位方向：同步事件后向下计数
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_HSPCLKDIV1;         // 高速时钟预分频：1分频 (100MHz)
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_CLKDIV1;               // 时基时钟预分频：1分频 (100MHz)
    EPwm1Regs.TBCTL.bit.SWFSYNC = TB_SWFSYNC_DISABLE;      // 软件强制同步：禁用
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;            // 同步输出选择：当计数器为零时产生同步信号


    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD_HALF;             // 设置 PWM 开关频率 因为是上下计数模式 所以除以 2
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;                   // 相位寄存器清零
    EPwm1Regs.TBCTR = 0x0000;                            // 时基计数器清零



    // 3、配置 Counter-Compare (CC) Submodule 共 4 个模块
    //----------------------------------------------------------------------------------------
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;          // CMPA 影子模式启用：使用影子寄存器
    //EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;        // CMPB 影子模式启用：使用影子寄存器
    //EPwm1Regs.CMPCTL2.bit.SHDWCMODE = CC_SHADOW;       // CMPC 影子模式启用：使用影子寄存器
    //EPwm1Regs.CMPCTL2.bit.SHDWDMODE = CC_SHADOW;       // CMPD 影子模式启用：使用影子寄存器

    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;        // CMPA 装载时机：计数器为零时装载
    //EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;      // CMPB 装载时机：计数器为零时装载
    //EPwm1Regs.CMPCTL2.bit.LOADCMODE = CC_CTR_ZERO;     // CMPC 装载时机：计数器为零时装载
    //EPwm1Regs.CMPCTL2.bit.LOADDMODE = CC_CTR_ZERO;     // CMPD 装载时机：计数器为零时装载

    // 注意 CMPA CMPB CMPC CMPD 之间的区别
    EPwm1Regs.CMPA.bit.CMPA = EPWM1_CMPA0;               // 设置比较器A初始值 (控制占空比)
//  EPwm1Regs.CMPB.bit.CMPB = EPWM1_CMPB0;               // 设置比较器B初始值 (控制占空比)
//  EPwm1Regs.CMPC = EPWM1_CMPC0;                       // 设置比较器C初始值 (控制占空比)
//  EPwm1Regs.CMPD = EPWM1_CMPD0;                       // 设置比较器D初始值 (控制占空比)



    // 4、配置 Action-Qualifier (AQ) Submodule
    //-----------------------------------------------------------------------------------------
    //  配置影子模式
//      EPwm1Regs.AQCTL.bit.SHDWAQAMODE = AQ_SHADOW;   // 设置AQCTLA使用影子寄存器模式
//      EPwm1Regs.AQCTL.bit.SHDWAQBMODE = AQ_SHADOW;   // 设置AQCTLB使用影子寄存器模式
//
//      EPwm1Regs.AQCTL.bit.LDAQAMODE = AQ_CTR_ZERO;   // 设置AQCTLA在计数器为零时(CTR=ZERO)加载影子寄存器
//      EPwm1Regs.AQCTL.bit.LDAQBMODE = AQ_CTR_ZERO;   // 设置AQCTLB在计数器为零时(CTR=ZERO)加载影子寄存器
//
//      EPwm1Regs.AQCTL.bit.LDAQASYNC = AQ_LOADMODE;   // 设置AQCTLA异步加载模式（仅在影子模式下有效）
//      EPwm1Regs.AQCTL.bit.LDAQBSYNC = AQ_LOADMODE;   // 设置AQCTLB异步加载模式（仅在影子模式下有效）

        // 配置立即模式 默认模式
        EPwm1Regs.AQCTL.bit.SHDWAQAMODE = AQ_IMMEDIATE;  // AQCTLA 立即模式：直接更新(无影子寄存器)
        EPwm1Regs.AQCTL.bit.SHDWAQBMODE = AQ_IMMEDIATE;  // AQCTLB 立即模式：直接更新(无影子寄存器)

       // 设置动作限定器A和B的输出动作
       //该部分可以设置为无动作，但是需要在改变占空比的中断中添加动作
       //TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1922
        EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;      // 在计数器为零时设置PWM1A输出高电平
        EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;    // 在CAU事件（Up-count时比较器A匹配）时清除PWM1A输出
//      EPwm1Regs.AQCTLA.all = AQ_NO_ACTION;    // 禁用AQCTLA所有动作（PWM1A保持原状态）

//      EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;      // 在计数器为零时设置PWM1B输出高电平
//      EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;    // 在CBU事件（Up-count时比较器B匹配）时清除PWM1B输出
//      EPwm1Regs.AQCTLB.all = AQ_NO_ACTION;    // 禁用AQCTLB所有动作（PWM1B保持原状态）



    // 5、配置 Dead-Band Generator (DB) Submodule
    // TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1923
    //----------------------------------------------------------------------------------------
    // IN_MODE    S4 = 0 S5 = 0
    // OUT_MODE   S0 = 1 S1 = 1
    // POLSEL     S2 = 0 S3 = 1
    // OUTSWAP    S6 = 0 S7 = 0
    // DEDB_MODE  S8 = 0

    // 设置死区的计数器时钟
    EPwm1Regs.DBCTL.bit.HALFCYCLE = DB_Fullcycle; // 死区计数器时钟 = TBCLK =100M

    // 设置死区的输入模式
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL; // 配置 EPWMxA下降沿和上升沿延迟的源

    // 设置死区的输出模式和极性
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // 使能下降沿和上升沿的延迟
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;      // EPWMxB is inverted

    //
    EPwm1Regs.DBCTL.bit.OUTSWAP = OUTSWAP_INVALID;   // S6 = 0 ，S7 = 0，输出不交换
    EPwm1Regs.DBCTL.bit.DEDB_MODE = B_Mode_DISABLE;  // S8 = 0 ，下降沿和上升沿通道由 S5 S4决定，输出到A B两个信号路径。
    //EPwm1Regs.DBCTL.bit.DEDB_MODE = B_Mode_ENABLE;   // S8 = 1 ，下降沿和上升沿通道由 S4决定 ，并仅输出到B信号路径。

    // 设置死区上升沿、下降沿的延时时长
    // 实际使用示波器测量正常
    EPwm1Regs.DBRED.bit.DBRED = EPWM1_DB; //  200ns
    EPwm1Regs.DBFED.bit.DBFED = EPWM1_DB; //  200ns

    // 6、配置 Event-Trigger (ET) Submodule
    //---------------------------------------------------------------------------------------

    // =============================================
    // 配置 SOCA 触发采样  TBCTR =ZERO
    // =============================================

    EPwm1Regs.ETSEL.bit.SOCAEN = EPWMxSOC_DISABLE;   // 禁用SOCA

    EPwm1Regs.ETSEL.bit.SOCASELCMP = ET_SELCMP_AB;   // 选择比较器组
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;       // 选择具体的事件触发 ADC (即在什么时刻触发 ADC)

    EPwm1Regs.ETPS.bit.SOCPSSEL = ET_PSSEL_3;       //选择脉冲数量的低档
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;           // SOCA周期：每个事件产生一个脉冲

    EPwm1Regs.ETCLR.bit.SOCA = ETCLR_CLEAR;         // 清除SOCA标志
    EPwm1Regs.ETSEL.bit.SOCAEN = EPWMxSOC_ENABLE;   // 使能 EPWMxSOCA

    // =============================================
    // 配置 SOCB 触发采样  TBCTR =TBPRD
    // =============================================
//    EPwm1Regs.ETSEL.bit.SOCBEN = EPWMxSOC_DISABLE;  // 禁用SOCB
//
//    EPwm1Regs.ETSEL.bit.SOCBSELCMP = ET_SELCMP_AB;  // 选择比较器组
//    EPwm1Regs.ETSEL.bit.SOCBSEL = ET_CTR_PRD;       // 选择具体的事件触发 ADC (即在什么时刻触发 ADC)
//
//    EPwm1Regs.ETPS.bit.SOCPSSEL = ET_PSSEL_3;       //选择脉冲数量的低档
//    EPwm1Regs.ETPS.bit.SOCBPRD = ET_1ST;           // SOCB周期：每个事件产生一个脉冲
//
//    EPwm1Regs.ETCLR.bit.SOCB = ETCLR_CLEAR;         // 清除SOCB标志
//    EPwm1Regs.ETSEL.bit.SOCBEN = EPWMxSOC_ENABLE;   // 使能 EPWMxSOCB

    // =============================================
    // 配置 INT 事件触发采样  COUNT DOWN TBCTR = CMPB
    // =============================================
//    EPwm1Regs.ETSEL.bit.INTSELCMP = ET_SELCMP_AB;   // 选择比较器组
//    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTRD_CMPBD;     // 选择具体的比较器事件：即在什么时刻触发 INT 事件
//
//    EPwm1Regs.ETPS.bit.INTPSSEL = ET_PSSEL_3;       // 选择脉冲数量的低档
//    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;             // 中断周期：每个事件产生一个中断
//    // 中断配置
//    EPwm1Regs.ETSEL.bit.INTEN = EPWMxINT_ENABLE;    // 使能 EPwmx_INT generation
//    EPwm1Regs.ETCLR.bit.INT = ETCLR_CLEAR;          // 清除中断标志

    // 7、Digital Compare (DC) Submodule 数字比较子模块
    // Digital Compare (DC) Submodule
    //----------------------------------------------------------------------------------------
    // 此部分配置用于高级保护功能，如过流、过压等故障检测和保护
    // 数字比较模块可以将外部比较器(CMPSS)的输出与ePWM模块连接，实现快速硬件保护

    // 配置 TRIP4 输入源为 CMPSS1 低电平跳闸信号 (CTRIP1L)
//    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX1 = EPWM_XBAR_Mux1_CMPSS1_CTRIPL; // 将TRIP4输入源设置为CMPSS1的低电平跳闸信号
//    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX1 = EPWM_XBAR_ENABLE;               // 使能TRIP4多路复用器输入

    // 配置 TRIP5 输入源为 CMPSS1 高电平跳闸信号 (CTRIP1H)
//    EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX0 = EPWM_XBAR_Mux0_CMPSS1_CTRIPH; // 将TRIP5输入源设置为CMPSS1的高电平跳闸信号
//    EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX0 = EPWM_XBAR_ENABLE;             // 使能TRIP5多路复用器输入

    // 设置数字比较(DCA)的触发源
//    EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_TRIPIN5;  // DCA高电平事件触发源选择TRIP5 (CMPSS1.CTRIPH)
//    EPwm1Regs.DCTRIPSEL.bit.DCALCOMPSEL = DC_TRIPIN4;  // DCA低电平事件触发源选择TRIP4 (CMPSS1.CTRIPL)

    // 设置 DCA 强制同步信号选择
//    EPwm1Regs.DCACTL.bit.EVT2FRCSYNCSEL = DC_EVT2FRCSYNCSEL_ASYNC; // EVT2不使用EPWMCLK作为强制同步信号(异步模式)
//    EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT1FRCSYNCSEL_ASYNC; // EVT1不使用EPWMCLK作为强制同步信号(异步模式)


    // 配置DC Blanking窗口(用于过滤噪声或开关瞬态引起的误触发)
//    EPwm1Regs.DCFCTL.bit.SRCSEL = DC_SRC_DCAEVT1;     // 选择DCAEVT1作为DC Blanking窗口的输入源
//    EPwm1Regs.DCFCTL.bit.PULSESEL = DC_PULSESEL_ZERO; // 配置DC Blanking窗口的触发脉冲: 计数器为零时触发
//    EPwm1Regs.DCFOFFSET = 0;                          // 配置DC Blanking窗口的偏移量: 0个TBCLK周期
//    EPwm1Regs.DCFWINDOW = 10;                         // 配置DC Blanking窗口的大小: 10个TBCLK周期
//    EPwm1Regs.DCFCTL.bit.BLANKE = DC_BLANK_ENABLE;    // 使能DC Blanking窗口功能

     //设置 DCA 事件选择
//    EPwm1Regs.DCACTL.bit.EVT2SRCSEL = DC_EVT2; // EVT2不使用DC Blanking窗口滤波(直接使用原始信号)
//    EPwm1Regs.DCACTL.bit.EVT1SRCSEL = DC_EVT_FILT; // EVT1使用DC Blanking窗口滤波后的信号


    // 8、Trip-Zone (TZ) Submodule 跳闸子模块
    //----------------------------------------------------------------------------------------
    // 配置数字比较器事件与跳闸操作的关联

    // 配置数字比较输出A事件1(DCAEVT1)的跳闸条件
//    EPwm1Regs.TZDCSEL.bit.DCAEVT1 = TZ_DCAL_HI ;            // DCAEVT1触发条件: DCAL为高电平(DCA低电平事件)
//    EPwm1Regs.TZSEL.bit.DCAEVT1 = TZ_DCAEVT1_ENABLE;        // 使能DCAEVT1作为一次性跳闸源

    // 配置数字比较输出A事件2(DCAEVT2)的跳闸条件
//    EPwm1Regs.TZDCSEL.bit.DCAEVT2 = TZ_DCAH_HI ;            // DCAEVT2触发条件: DCAH为高电平(DCA高电平事件)
//    EPwm1Regs.TZSEL.bit.DCAEVT2 = TZ_DCAEVT2_DISABLE;       // 禁用DCAEVT2作为循环跳闸(CBC)源

    // 配置跳闸事件发生时对ePWM输出的操作
//    EPwm1Regs.TZCTL.bit.DCAEVT1 = TZ_HIZ;          // DCAEVT1发生时: 将EPWM输出设置为高阻态
//    EPwm1Regs.TZCTL.bit.DCAEVT2 = TZ_NO_CHANGE;    // DCAEVT2发生时: 不改变EPWM输出(已禁用)

    // 配置全局跳闸事件对ePWM输出的操作
//    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;   // 跳闸时强制EPWMxA输出低电平(安全状态)
//    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;   // 跳闸时强制EPWMxB输出低电平(安全状态)

    // 启动所有ePWM时基时钟同步
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // 启动所有ePWM时基时钟同步，开始PWM生成

    EDIS;     // 禁止访问受保护的寄存器
}


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_ePWM2(void)
 功能描述：    配置 ePWM2
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：   a.配置为上下计数模式，产生对称PWM波形
             b.启用死区生成，防止桥式电路直通
             c.配置事件触发用于ADC启动
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW             创建
********************************************************************************/
/*
 * 配置步骤：
 * ----------------------------------------------------------
 * 1、配置前停止 ePWM 外设时钟
 * 2、配置 Time-Base (TB) Submodule
 * 3、配置 Counter-Compare (CC) Submodule
 * 4、配置 Action-Qualifier (AQ) Submodule
 * 5、配置 Dead-Band Generator (DB) Submodule
 * 6、配置 Event-Trigger (ET) Submodule
 * 7、配置Digital Compare (DC) Submodule
 * 8、配置Trip-Zone (TZ) Submodule
 * ----------------------------------------------------------
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1877
 *----------------------------------------------------------*/
void Init_ePWM2(void){

    EALLOW;             // 允许访问受保护的寄存器

    // 1、配置前停止 ePWM 外设时钟
    //----------------------------------------------------------------------------
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; // 停止所有 ePWM 时基时钟同步，确保安全配置

    // 2、配置 Time-Base (TB) Submodule
    //--------------------------------------------------------------------------------------
    /*
     * 时钟计算:
     * SYSCLK = EPWMCLK = 100M
     * TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)
     * TBCLK = 100M = 100M/1*1=100M
     *
     *TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1890
     * PWM频率计算 (上下计数模式):
     * PWM频率 = TBCLK / (2 × TBPRD)
     * 20kHz = 100MHz / (2 × TBPRD)
     * 因此 TBPRD = 2500
     */

    EPwm2Regs.TBCTL.bit.FREE_SOFT = TB_FREE;               // 仿真模式：自由运行，仿真挂起时PWM继续运行
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;         // 上下计数模式
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;                 // 周期寄存器装载模式：从影子寄存器装载

    // 时间基计数器同步
    /* ------------------------------------------------------
    ePWM模块能够自动同步到另一个ePWM模块的时间基。可以将超前或滞后相位控制添加到由不同的ePWM模块产生的波形中，以使其同步。
    在向上向下计数模式下，TBCTL[PSHDIR]位在同步事件后立即配置时基计数器的方向。
    新的方向与同步事件发生之前的方向无关。
    在向上计数或向下计数模式下，将忽略TBCTL[PSHDIR]位。
    * ------------------------------------------------------*/
    EPwm2Regs.TBCTL.bit.PHSEN = TB_PHSEN_ENABLE;           // 相位寄存器使能
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_PHSDIR_UP;             // 相位方向：同步事件后向上计数，上下计数模式有效

    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_HSPCLKDIV1;         // 高速时钟预分频：1分频 (100MHz)
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_CLKDIV1;               // 时基时钟预分频：1分频 (100MHz)
    EPwm2Regs.TBCTL.bit.SWFSYNC = TB_SWFSYNC_DISABLE;      // 软件强制同步：禁用
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_EPWMxSYNCI_SWFSYNC;  // 同步输出信号来源于输入的同步信号(EPWMxSYNCI)或软件强制同步(SWFSYNC)

    EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD_HALF;   // 设置 PWM 开关频率 因为是上下计数模式 所以除以 2
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0002;         // 设置相位偏移计数器
    EPwm2Regs.TBCTR = 0x0000;                   // 清零时基计数器

    // 3、配置 Counter-Compare (CC) Submodule 共 4 个模块
    //----------------------------------------------------------------------------
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // CMPA 影子模式启用：使用影子寄存器
    //EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;       // CMPB 影子模式启用：使用影子寄存器
    //EPwm2Regs.CMPCTL2.bit.SHDWCMODE = CC_SHADOW;      // CMPC 影子模式启用：使用影子寄存器
    //EPwm2Regs.CMPCTL2.bit.SHDWDMODE = CC_SHADOW;      // CMPD 影子模式启用：使用影子寄存器



    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;       // CMPA 装载时机：计数器为零时装载
    //EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;     // CMPB 装载时机：计数器为零时装载
    //EPwm2Regs.CMPCTL2.bit.LOADCMODE = CC_CTR_ZERO;    // CMPC 装载时机：计数器为零时装载
    //EPwm2Regs.CMPCTL2.bit.LOADDMODE = CC_CTR_ZERO;    // CMPD 装载时机：计数器为零时装载

    // 注意 CMPA CMPB CMPC CMPD 之间的区别
    EPwm2Regs.CMPA.bit.CMPA = EPWM2_CMPA0;               // 设置比较器A初始值 (控制占空比)
    //EPwm2Regs.CMPB.bit.CMPB = EPWM2_CMPB0;    // 设置比较器B初始值 (控制占空比)
    //EPwm2Regs.CMPC = EPWM2_CMPC0;             // 设置比较器C初始值 (控制占空比)
    //EPwm2Regs.CMPD = EPWM2_CMPD0;             // 设置比较器D初始值 (控制占空比)

    // 4、配置 Action-Qualifier (AQ) Submodule
    //----------------------------------------------------------------------------
    // 配置影子模式
//    EPwm2Regs.AQCTL.bit.SHDWAQAMODE = AQ_SHADOW;  // 设置AQCTLA使用影子寄存器模式
//    EPwm2Regs.AQCTL.bit.SHDWAQBMODE = AQ_SHADOW;  // 设置AQCTLB使用影子寄存器模式
//
//    EPwm2Regs.AQCTL.bit.LDAQAMODE = AQ_CTR_PRD;   // 设置AQCTLA在计数器为零时(CTR=TBPRD)加载影子寄存器
//    EPwm2Regs.AQCTL.bit.LDAQBMODE = AQ_CTR_PRD;   // 设置AQCTLB在计数器为零时(CTR=TBPRD)加载影子寄存器
//
//    EPwm2Regs.AQCTL.bit.LDAQASYNC = AQ_LOADMODE;  // 设置AQCTLA异步加载模式（仅在影子模式下有效）
//    EPwm2Regs.AQCTL.bit.LDAQBSYNC = AQ_LOADMODE;  // 设置AQCTLA异步加载模式（仅在影子模式下有效）

    // 配置立即模式 默认模式
    EPwm2Regs.AQCTL.bit.SHDWAQAMODE = AQ_IMMEDIATE;  // AQCTLA 立即模式：直接更新(无影子寄存器)
    EPwm2Regs.AQCTL.bit.SHDWAQBMODE = AQ_IMMEDIATE;  // AQCTLB 立即模式：直接更新(无影子寄存器)

    // 设置动作限定器A和B的输出动作
    //该部分可以设置为无动作，但是需要在改变占空比的中断中添加动作
    //只设置PWMA的动作也可以输出PWMB是因为DB子模块将PWMA反相得到PWMB
    //TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1922
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;       // 在计数器为零时设置PWM2A输出高电平
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;     // 在CAU事件（Up-count时比较器A匹配）时清除PWM2A输出
//  EPwm2Regs.AQCTLA.all = AQ_NO_ACTION;    //禁用AQCTLB所有动作（PWM2B保持原状态）



    //EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;      // 在计数器为零时设置PWM2B输出高电平
    //EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;    // 在CBU事件（Up-count时比较器A匹配）时清除PWM2B输出
    //EPwm2Regs.AQCTLB.all = AQ_NO_ACTION;    //禁用AQCTLB所有动作（PWM2B保持原状态）

    // 5、配置 Dead-Band Generator (DB) Submodule
    // TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page1923
    //----------------------------------------------------------------------------------------
    // IN_MODE    S4 = 0 S5 = 0
    // OUT_MODE   S0 = 1 S1 = 1
    // POLSEL     S2 = 0 S3 = 1
    // OUTSWAP    S6 = 0 S7 = 0
    // DEDB_MODE  S8 = 0

    // 设置死区的计数器时钟
    EPwm2Regs.DBCTL.bit.HALFCYCLE = DB_Fullcycle; // 死区计数器时钟 = TBCLK =100M

    // 设置死区的输入模式
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL; // 配置 EPWMxA 下降沿和上升沿延迟的源

    // 设置死区的输出模式和极性
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // 使能下降沿和上升沿的延迟
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;      // EPWMxB is inverted

    //
    EPwm2Regs.DBCTL.bit.OUTSWAP = OUTSWAP_INVALID;   // S6 = 0 ，S7 = 0，输出不交换
    EPwm2Regs.DBCTL.bit.DEDB_MODE = B_Mode_DISABLE;  // S8 = 0 ，下降沿和上升沿通道由 S5 S4决定，输出到A B两个信号路径。
    //EPwm2Regs.DBCTL.bit.DEDB_MODE = B_Mode_ENABLE;   // S8 = 1 ，下降沿和上升沿通道由 S4决定 ，并仅输出到B信号路径。

    // 设置死区上升沿、下降沿的延时时长
    EPwm2Regs.DBRED.bit.DBRED = EPWM2_DB; // 200ns
    EPwm2Regs.DBFED.bit.DBFED = EPWM2_DB; // 200ns

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    EDIS;    // 禁止访问受保护的寄存器
}
