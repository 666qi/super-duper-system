//###########################################################################
//
// FILE:   f28004x_gpio.c
//
// TITLE:  f28004x GPIO module support functions
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
#include "f28004x_device.h"
#include "f28004x_examples.h"

//
// Low-level functions for GPIO configuration
//

//
// InitGpio - Sets all pins to be muxed to GPIO in input mode with pull-ups
// enabled.Also resets CPU control to CPU1 and disables open drain
// and polarity inversion and sets the qualification to synchronous.
// Also unlocks all GPIOs. Only one CPU should call this function.
//
void
InitGpio()
{
    volatile Uint32 *gpioBaseAddr;
    Uint16 regOffset;

    //
    // Disable pin locks
    //
    EALLOW;
    GpioCtrlRegs.GPALOCK.all = 0x00000000;
    GpioCtrlRegs.GPBLOCK.all = 0x00000000;
    GpioCtrlRegs.GPHLOCK.all = 0x00000000;

    //
    // Fill all registers with zeros. Writing to each register separately
    // for three GPIO modules would make this function *very* long.
    // Fortunately, we'd be writing them all with zeros anyway,
    // so this saves a lot of space.
    //
    gpioBaseAddr = (Uint32 *)&GpioCtrlRegs;
    for (regOffset = 0; regOffset < sizeof(GpioCtrlRegs)/2; regOffset++)
    {
        //
        // Must avoid enabling pull-ups on all pins. GPyPUD is offset
        // 0x0C in each register group of 0x40 words. Since this is a
        // 32-bit pointer, the addresses must be divided by 2.
        //
        // Also, to avoid changing pin muxing of the emulator pins to regular
        // GPIOs, skip GPBMUX1 (0x46) and GPBGMUX1 (0x60).
        //
        if ((regOffset % (0x40/2) != (0x0C/2)) && (regOffset != (0x46/2)) &&
            (regOffset != (0x60/2)))
        {
            gpioBaseAddr[regOffset] = 0x00000000;
        }
    }

    gpioBaseAddr = (Uint32 *)&GpioDataRegs;
    for (regOffset = 0; regOffset < sizeof(GpioDataRegs)/2; regOffset++)
    {
        gpioBaseAddr[regOffset] = 0x00000000;
    }

    EDIS;
}

//
// GPIO_SetupPinMux - Set the peripheral muxing for the specified pin.
// The appropriate parameters can be found in the pinout spreadsheet.
//
void
GPIO_SetupPinMux(Uint16 gpioNumber, Uint16 cpu, Uint16 muxPosition)
{
    volatile Uint32 *gpioBaseAddr;
    volatile Uint32 *mux, *gmux, *csel;
    Uint16 pin32, pin16, pin8;

    pin32 = gpioNumber % 32;
    pin16 = gpioNumber % 16;
    pin8 = gpioNumber % 8;
    gpioBaseAddr = (Uint32 *)&GpioCtrlRegs + (gpioNumber/32)*GPY_CTRL_OFFSET;

    //
    // Sanity check for valid cpu and peripheral values
    //
    if (cpu > GPIO_MUX_CPU1CLA || muxPosition > 0xF)
    {
        return;
    }

    //
    // Create pointers to the appropriate registers. This is a workaround
    // for the way GPIO registers are defined. The standard definition
    // in the header file makes it very easy to do named accesses of one
    // register or bit, but hard to do arbitrary numerical accesses. It's
    // easier to have an array of GPIO modules with identical registers,
    // including arrays for multi-register groups like GPyCSEL1-4. But
    // the header file doesn't define anything we can turn into an array,
    // so manual pointer arithmetic is used instead.
    //
    mux = gpioBaseAddr + GPYMUX + pin32/16;
    gmux = gpioBaseAddr + GPYGMUX + pin32/16;
    csel = gpioBaseAddr + GPYCSEL + pin32/8;

    //
    // Now for the actual function
    //
    EALLOW;

    //
    // To change the muxing, set the peripheral mux to 0/GPIO first to avoid
    // glitches, then change the group mux, then set the peripheral mux to
    // its target value. Finally, set the CPU select. This procedure is
    // described in the TRM. Unfortunately, since we don't know the pin in
    // advance we can't hardcode a bitfield reference, so there's some tricky
    // bit twiddling here.
    //
    *mux &= ~(0x3UL << (2*pin16));
    *gmux &= ~(0x3UL << (2*pin16));
    *gmux |= (Uint32)((muxPosition >> 2) & 0x3UL) << (2*pin16);
    *mux |= (Uint32)(muxPosition & 0x3UL) << (2*pin16);

    *csel &= ~(0x3L << (4*pin8));
    *csel |= (Uint32)(cpu & 0x3L) << (4*pin8);

    //
    // WARNING: This code does not touch the analog mode select registers.
    //

    EDIS;
}

//
// GPIO_SetupPinOptions - Setup up the GPIO input/output options for the
// specified pin. The flags are a 16-bit mask produced by ORing together
// options. For input pins, the valid flags are:
// GPIO_PULLUP  Enable pull-up
// GPIO_INVERT  Enable input polarity inversion
// GPIO_SYNC  Synchronize the input latch to PLLSYSCLK
//              (default -- you don't need to specify this)
// GPIO_QUAL3  Use 3-sample qualification
// GPIO_QUAL6  Use 6-sample qualification
// GPIO_ASYNC  Do not use synchronization or qualification
// (Note: only one of SYNC, QUAL3, QUAL6, or ASYNC is allowed)
//
// For output pins, the valid flags are:
// GPIO_OPENDRAIN  Output in open drain mode
// GPIO_PULLUP    If open drain enabled, also enable the pull-up
// and the input qualification flags (SYNC/QUAL3/QUAL6/SYNC) listed above.
//
// With no flags, the default input state is synchronous with no
// pull-up or polarity inversion. The default output state is
// the standard digital output.
//
void
GPIO_SetupPinOptions(Uint16 gpioNumber, Uint16 output, Uint16 flags)
{
    volatile Uint32 *gpioBaseAddr;
    volatile Uint32 *dir, *pud, *inv, *odr, *qsel;
    Uint32 pin32, pin16, pinMask, qual;

    pin32 = gpioNumber % 32;
    pin16 = gpioNumber % 16;
    pinMask = 1UL << pin32;
    gpioBaseAddr = (Uint32 *)&GpioCtrlRegs + (gpioNumber/32)*GPY_CTRL_OFFSET;

    //
    // Create pointers to the appropriate registers. This is a workaround
    // for the way GPIO registers are defined. The standard definition
    // in the header file makes it very easy to do named accesses of one
    // register or bit, but hard to do arbitrary numerical accesses. It's
    // easier to have an array of GPIO modules with identical registers,
    // including arrays for multi-register groups like GPyQSEL1-2. But
    // the header file doesn't define anything we can turn into an array,
    // so manual pointer arithmetic is used instead.
    //
    dir = gpioBaseAddr + GPYDIR;
    pud = gpioBaseAddr + GPYPUD;
    inv = gpioBaseAddr + GPYINV;
    odr = gpioBaseAddr + GPYODR;
    qsel = gpioBaseAddr + GPYQSEL + pin32/16;

    EALLOW;

    //
    // Set the data direction
    //
    *dir &= ~pinMask;
    if (output == 1)
    {
        //
        // Output, with optional open drain mode and pull-up
        //
        *dir |= pinMask;

        //
        // Enable open drain if necessary
        //
        if (flags & GPIO_OPENDRAIN)
        {
            *odr |= pinMask;
        }
        else
        {
            *odr &= ~pinMask;
        }

        //
        // Enable pull-up if necessary. Open drain mode must be active.
        //
        if (flags & (GPIO_OPENDRAIN | GPIO_PULLUP))
        {
            *pud &= ~pinMask;
        }
        else
        {
            *pud |= pinMask;
        }
    }

    else
    {
        //
        // Input, with optional pull-up, qualification, and polarity inversion
        //
        *dir &= ~pinMask;

        //
        // Enable pull-up if necessary
        //
        if (flags & GPIO_PULLUP)
        {
            *pud &= ~pinMask;
        }
        else
        {
            *pud |= pinMask;
        }

        //
        // Invert polarity if necessary
        //
        if (flags & GPIO_INVERT)
        {
            *inv |= pinMask;
        }
        else
        {
            *inv &= ~pinMask;
        }
    }

    //
    // Extract the qualification parameter and load it into the register.
    // This is also needed for open drain outputs, so we might as well do it
    // all the time.
    //
    qual = (flags & GPIO_ASYNC) / GPIO_QUAL3;
    *qsel &= ~(0x3L << (2 * pin16));
    if (qual != 0x0)
    {
        *qsel |= qual << (2 * pin16);
    }

    EDIS;
}

//
// GPIO_SetupLock - Enable or disable the GPIO register bit lock for the
// specified pin. The valid flags are:
// GPIO_UNLOCK  Unlock the pin setup register bits for the specified pin
// GPIO_LOCK   Lock the pin setup register bits for the specified pin
//
void
GPIO_SetupLock(Uint16 gpioNumber, Uint16 flags)
{
    volatile Uint32 *gpioBaseAddr;
    volatile Uint32 *lock;
    Uint32 pin32, pinMask;

    pin32 = gpioNumber % 32;
    pinMask = 1UL << pin32;
    gpioBaseAddr = (Uint32 *)&GpioCtrlRegs + (gpioNumber/32)*GPY_CTRL_OFFSET;

    //
    // Create pointers to the appropriate registers. This is a workaround
    // for the way GPIO registers are defined. The standard definition
    // in the header file makes it very easy to do named accesses of one
    // register or bit, but hard to do arbitrary numerical accesses. It's
    // easier to have an array of GPIO modules with identical registers,
    // including arrays for multi-register groups like GPyQSEL1-2. But
    // the header file doesn't define anything we can turn into an array,
    // so manual pointer arithmetic is used instead.
    //
    lock = gpioBaseAddr + GPYLOCK;

    EALLOW;

    if(flags)
    {
        //
        // Lock the pin
        //
        *lock |= pinMask;
    }

    else
    {
        //
        // Unlock the pin
        //
        *lock &= ~pinMask;
    }
    EDIS;
}

//
// External interrupt setup
//
void
GPIO_SetupXINT1Gpio(Uint16 gpioNumber)
{
    EALLOW;
    InputXbarRegs.INPUT4SELECT = gpioNumber;       // Set XINT1 source to GPIO-pin
    EDIS;
}

void
GPIO_SetupXINT2Gpio(Uint16 gpioNumber)
{
    EALLOW;
    InputXbarRegs.INPUT5SELECT = gpioNumber;       // Set XINT2 source to GPIO-pin
    EDIS;
}

void
GPIO_SetupXINT3Gpio(Uint16 gpioNumber)
{
    EALLOW;
    InputXbarRegs.INPUT6SELECT = gpioNumber;       // Set XINT3 source to GPIO-pin
    EDIS;
}
void
GPIO_SetupXINT4Gpio(Uint16 gpioNumber)
{
    EALLOW;
    InputXbarRegs.INPUT13SELECT = gpioNumber;      // Set XINT4 source to GPIO-pin
    EDIS;
}

void
GPIO_SetupXINT5Gpio(Uint16 gpioNumber)
{
    EALLOW;
    InputXbarRegs.INPUT14SELECT = gpioNumber;     // Set XINT5 source to GPIO-pin
    EDIS;
}

//
// GPIO_ReadPin - Read the GPyDAT register bit for the specified pin.
// Note that this returns the actual state of the pin, not the state of the
// output latch.
//
Uint16
GPIO_ReadPin(Uint16 gpioNumber)
{
    volatile Uint32 *gpioDataReg;
    Uint16 pinVal;

    gpioDataReg = (volatile Uint32 *)&GpioDataRegs + (gpioNumber/32)*GPY_DATA_OFFSET;
    pinVal = (gpioDataReg[GPYDAT] >> (gpioNumber % 32)) & 0x1;

    return pinVal;
}

//
// GPIO_WritePin - Set the GPyDAT register bit for the specified pin.
//
void
GPIO_WritePin(Uint16 gpioNumber, Uint16 outVal)
{
    volatile Uint32 *gpioDataReg;
    Uint32 pinMask;

    gpioDataReg = (volatile Uint32 *)&GpioDataRegs + (gpioNumber/32)*GPY_DATA_OFFSET;
    pinMask = 1UL << (gpioNumber % 32);

    if (outVal == 0)
    {
        gpioDataReg[GPYCLEAR] = pinMask;
    }

    else
    {
        gpioDataReg[GPYSET] = pinMask;
    }
}



// 自己添加的函数
/* --------------------------------------------------------------
 * 用户根据自己的实际需求修改
 * --------------------------------------------------------------*/


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_Led(void)
 功能描述：    配置 GPIO23 和 GPIO34引脚为输出模式，用于控制 LED
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
 ********************************************************************************/
/* ----------------------------------------------------------------------------
 * GPIO 配置步骤：
 * 1. 使能/禁用内部上拉电阻 (GPxPUD)
 * 2. 配置开漏输出模式 (GPxODR) - 可选
 * 3. 配置多路复用器选择 GPIO 功能 (GPxGMUX, GPxMUX)
 * 4. 设置引脚方向为输出 (GPxDIR)
 * 5. 初始化输出电平 (GPxSET/GPxCLEAR)
 *
 * 注意：GPIO23位于GPIOA组，GPIO34位于GPIOB组
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page936
 * ----------------------------------------------------------------------------*/
void Init_Led(void){

    EALLOW; // 允许写入受保护的寄存器

    // 配置红色LED控制引脚(GPIO23)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = PUD_ENABLE;     // 使能GPIO23上拉电阻

    GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = 0;            // 选择GPIO23为通用I/O模式(第0组)
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;             // 选择GPIO23功能(0=GPIO, 1=外设功能)

    GpioCtrlRegs.GPADIR.bit.GPIO23 = PIN_OUTPUT;     // 设置GPIO23为输出模式
    GpioDataRegs.GPASET.bit.GPIO23 = 1;              // 灭灯（输出高电平）
    //GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;          // 亮灯（输出低电平）

    // 配置绿色LED控制引脚(GPIO34)
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = PUD_ENABLE;     // 使能GPIO34上拉电阻
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;            // 选择GPIO34为通用I/O模式(第0组)
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;             // 选择GPIO34功能(0=GPIO, 1=外设功能)

    GpioCtrlRegs.GPBDIR.bit.GPIO34 = PIN_OUTPUT;     // 设置GPIO34为输出模式
    GpioCtrlRegs.GPBODR.bit.GPIO34 = 1;              // 设置GPIO34为开漏输出模式
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;              // 灭灯（输出高电平）
    //GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;          // 亮灯（输出低电平）

    EDIS;                                            // 禁止写入受保护的寄存器，恢复保护状态
}


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_InputIO(void)
 功能描述：    初始化GPIO10和GPIO11作为输入引脚，配置相关输入特性
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
 ********************************************************************************/
/* -----------------------------------------------------------------------------
 * GPIO 输入配置步骤：
 * ----------------------------------------------------------------------------
 * 1. 配置上拉电阻 (GPxPUD)
 * 2. 配置多路复用器选择 GPIO 功能 (GPxGMUX, GPxMUX)
 * 3. 设置引脚方向为输入 (GPxDIR)
 * 4. 配置输入信号极性反转 (GPxINV) - 可选
 * 5. 配置输入信号滤波和采样 (GPxCTRL, GPxQSEL) - 可选
 * 6. 配置低功耗模式唤醒源 (GPIOLPMSEL) - 可选
 * 7. 配置外部中断源 (XINTnCR, INPUTxSELECT) - 可选
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page938
 * ----------------------------------------------------------------------------*/
void Init_InputIO(void){

    EALLOW; // 允许访问受保护的寄存器

    // 配置 GPIO10 和 GPIO11 为上拉输入模式
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = PUD_ENABLE;     // 使能 GPIO10 内部上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = PUD_ENABLE;     // 使能 GPIO11 内部上拉电阻

    // 配置 GPIO 功能（非外设功能）
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;            // 选择 GPIO10 功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;             // 配置 GPIO10 为通用 I/O 功能
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;            // 选择 GPIO11 功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;             // 配置 GPIO11 为通用 I/O 功能

    // 设置引脚方向为输入
    GpioCtrlRegs.GPADIR.bit.GPIO10 = PIN_INPUT;     // 设置 GPIO10 为输入模式
    GpioCtrlRegs.GPADIR.bit.GPIO11 = PIN_INPUT;     // 设置 GPIO11 为输入模式

    // 可选：输入信号极性反转
    //GpioCtrlRegs.GPAINV.bit.GPIO10 = GPIO_INV_ENABLE; // 使能 GPIO10 输入信号反相
    //GpioCtrlRegs.GPAINV.bit.GPIO11 = GPIO_INV_ENABLE; // 使能 GPIO11 输入信号反相

    // 配置输入信号滤波（防抖动）
    /*
     * QUALPRD 寄存器设置采样周期：
     * 0x00: 采样周期 = 0 个系统时钟周期 (无滤波)
     * 0x01: 采样周期 = 2 个系统时钟周期
     * 0x02: 采样周期 = 4 个系统时钟周期
     * ...
     * 0xFF: 采样周期 = 510 个系统时钟周期
     * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page952
     */
    GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 10;                // 设置 GPIO8-15 的采样周期为 20 个 SYSCLK
    GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = GPIO_QSEL_3_sample; // 配置 GPIO10 为 3 次采样滤波
    GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 10;                // 设置 GPIO8-15 的采样周期为 20 个 SYSCLK
    GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = GPIO_QSEL_3_sample; // 配置 GPIO11 为 3 次采样滤波

    // 可选：低功耗模式唤醒配置
    //CpuSysRegs.GPIOLPMSEL0.bit.GPIO10 = GPIO_LPMSEL_ENABLE; // 使能 GPIO10 作为低功耗模式唤醒源
    //CpuSysRegs.GPIOLPMSEL0.bit.GPIO11 = GPIO_LPMSEL_ENABLE; // 使能 GPIO11 作为低功耗模式唤醒源

    // 配置外部中断 (XINT5)
    // 注意：外部中断配置需要两个步骤：
    // 1. 通过 XINTnCR 寄存器使能中断并配置极性
    // 2. 通过 INPUTxSELECT 寄存器将 GPIO 引脚连接到输入 X-BAR
    XintRegs.XINT5CR.bit.POLARITY = XINT_POLARITY_negative;    // 设置 XINT5 为下降沿触发
    XintRegs.XINT5CR.bit.ENABLE = XINT_ENABLE;                 // 使能 XINT5 中断

    // 将 GPIO10 连接到输入 X-BAR 的 INPUT14，进而连接到 XINT5
    InputXbarRegs.INPUT14SELECT = INPUT_XBAR_GPIO10;

    // 注意：还需在主程序中使能相应的 PIE 中断 (参考手册 3.5.5 节 PIE 通道映射)

    EDIS; // 禁止访问受保护的寄存器
}


// =============================================================================
/*******************************************************************************
 函数名称：    void Init_SpecialGpio(void)
 功能描述：    配置 GPIO0-GPIO3 为输出模式，并初始化为低电平，PWM输出
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2025/08/27    V1.0           LW              创建
 ********************************************************************************/
/* *******************************************************
 * 初始化一些系统使用的GPIO
 *
 * 配置 GPIOA 输出步骤如下
 * ----------------------------------------------------------------------------
 * GPIO 输出配置步骤：
 * 1. 配置上拉/下拉电阻 (GPxPUD)
 * 2. 配置开漏输出模式 (GPxODR) - 可选
 * 3. 配置多路复用器选择 GPIO 功能 (GPxGMUX, GPxMUX)
 * 4. 设置引脚方向为输出 (GPxDIR)
 * 5. 初始化输出电平 (GPxSET/GPxCLEAR)
 * TI《TMS320F28004x Real-Time Microcontrollers》技术文档  page936
 * ----------------------------------------------------------------------------
 * *******************************************************/
void Init_SpecialGpio(void){

    EALLOW;     // 允许访问受保护的寄存器

    // 配置 GPIO0 GPIO1 GPIO2 GPIO3
    // 禁用内部上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = PUD_DISABLE;  // 禁用 GPIO0 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = PUD_DISABLE;  // 禁用 GPIO1 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = PUD_DISABLE;  // 禁用 GPIO2 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = PUD_DISABLE;  // 禁用 GPIO3 上拉电阻

    // 配置为通用 GPIO 功能（非外设功能）
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;    // 选择 GPIO0 功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;     // 配置 GPIO0 为通用 I/O 功能

    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;    // 选择 GPIO1 功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;     // 配置 GPIO1 为通用 I/O 功能

    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;    // 选择 GPIO2 功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;     // 配置 GPIO2 为通用 I/O 功能

    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;    // 选择 GPIO3 功能组 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;     // 配置 GPIO3 为通用 I/O 功能

    // 设置引脚方向为输出模式
    GpioCtrlRegs.GPADIR.bit.GPIO0 = PIN_OUTPUT;  // 设置 GPIO0 为输出模式
    GpioCtrlRegs.GPADIR.bit.GPIO1 = PIN_OUTPUT;  // 设置 GPIO1 为输出模式
    GpioCtrlRegs.GPADIR.bit.GPIO2 = PIN_OUTPUT;  // 设置 GPIO2 为输出模式
    GpioCtrlRegs.GPADIR.bit.GPIO3 = PIN_OUTPUT;  // 设置 GPIO3 为输出模式

    // 初始化所有 GPIO 输出低电平
    GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;  // 设置 GPIO0 输出低电平
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;  // 设置 GPIO1 输出低电平
    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;  // 设置 GPIO2 输出低电平
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;  // 设置 GPIO3 输出低电平

    EDIS;    // 禁止访问受保护的寄存器
}

//
// End of File
//

