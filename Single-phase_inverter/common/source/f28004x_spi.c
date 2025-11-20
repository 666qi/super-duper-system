//###########################################################################
//
// FILE:   f28004x_Spi.c
//
// TITLE:  f28004x SPI Initialization & Support Functions.
//
//#############################################################################
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
//#############################################################################

//
// Included Files
//
#include "f28004x_device.h"      // Headerfile Include File
#include "f28004x_examples.h"    // Examples Include File

//
// Calculate BRR: 7-bit baud rate register value
// SPI CLK freq = 500 kHz
// LSPCLK freq  = CPU freq / 4  (by default)
// BRR          = (LSPCLK freq / SPI CLK freq) - 1
//
#if CPU_FRQ_100MHZ
#define SPI_BRR        ((100E6 / 4) / 500E3) - 1
#endif



//
// InitSPI - This function initializes the SPI to a known state
//
void Init_SPIA(void)
{
    //
    // Initialize SPI-A
    //
    // Set reset low before configuration changes
    // Clock polarity (0 == rising, 1 == falling)
    // 16-bit character
    // Enable loop-back
    //
    SpiaRegs.SPICCR.bit.SPISWRESET = SPICCR_SPISWRESET_RESET;//SPI 复位
    SpiaRegs.SPICCR.bit.CLKPOLARITY = SPICCR_CLKPOLARITY_0;// 时钟极性
    SpiaRegs.SPICCR.bit.SPICHAR = SPICCR_SPICHAR_LENGTH_16;// 字节长度
    SpiaRegs.SPICCR.bit.SPILBK = SPICCR_LOOPBKENA_ENABLED;// 回环模式使能

    //
    // Enable master (0 == slave, 1 == master)
    // Enable transmission (Talk)
    // Clock phase (0 == normal, 1 == delayed)
    // SPI interrupts are disabled
    //
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = SPICTL_MASTER; // SPI 主模式使能
    SpiaRegs.SPICTL.bit.TALK = SPICTL_TALK_ENABLED; // SPI 传输使能
    SpiaRegs.SPICTL.bit.CLK_PHASE = SPICTL_CLK_PHASE_NORMAL;// 时钟相位
    SpiaRegs.SPICTL.bit.SPIINTENA = SPICTL_SPIINTENA_DISABLED;// SPI 中断禁止

    //
    // Set the baud rate
    //
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;

    //
    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI
    //
    SpiaRegs.SPIPRI.bit.FREE = SPIPRI_FREE_RUN;

    //
    // Release the SPI from reset
    //
    SpiaRegs.SPICCR.bit.SPISWRESET = SPICCR_SPISWRESET_REENABLED;// 释放 SPI 复位
}

//
// InitSpiGpio - This function initializes GPIO pins to function as SPI pins.
//               Each GPIO pin can be configured as a GPIO pin or up to 3
//               different peripheral functional pins. By default all pins come
//               up as GPIO inputs after reset.
//
//               Caution:
//               For each SPI peripheral
//               Only one GPIO pin should be enabled for SPISOMO operation.
//               Only one GPIO pin should be enabled for SPISOMI operation.
//               Only one GPIO pin should be enabled for SPICLK  operation.
//               Only one GPIO pin should be enabled for SPISTE  operation.
//               Comment out other unwanted lines.
//
void InitSpiGpio()
{
    InitSpiaGpio();
}

//
// InitSpiaGpio - Initialize SPIA GPIOs
//
void InitSpiaGpio()
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    //
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = PUD_ENABLE; // Enable pull-up on GPIO16 (SPISIMOA)
    // GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;  // Enable pull-up on GPIO5 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = PUD_ENABLE; // Enable pull-up on GPIO17 (SPISOMIA)
    // GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;  // Enable pull-up on GPIO3 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = PUD_ENABLE; // Enable pull-up on GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = PUD_ENABLE; // Enable pull-up on GPIO19 (SPISTEA)

    //
    // Set qualification for selected pins to asynch only
    //
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = GPIO_QSEL_Asynchronous; // Asynch input GPIO16 (SPISIMOA)
    // GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 3;  // Asynch input GPIO5 (SPISIMOA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = GPIO_QSEL_Asynchronous; // Asynch input GPIO17 (SPISOMIA)
    // GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 3;  // Asynch input GPIO3 (SPISOMIA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = GPIO_QSEL_Asynchronous; // Asynch input GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = GPIO_QSEL_Asynchronous; // Asynch input GPIO19 (SPISTEA)

    //
    //Configure SPI-A pins using GPIO regs
    //
    // This specifies which of the possible GPIO pins will be SPI functional
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
    // GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 2;  // Configure GPIO5 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
    // GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 2;  // Configure GPIO3 as SPISOMIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA

    EDIS;
}

void Init_SPIAFIFO(void)
{
    //
    // Initialize SPI FIFO registers
    //

    SpiaRegs.SPIFFTX.bit.SPIRST = SPIFFTX_SPIRST_RESET;// 复位 SPI 通道
    SpiaRegs.SPIFFTX.bit.TXFIFO = SPIFFTX_TXFIFO_HOLDRESET;// 复位 TXFIFO

    SpiaRegs.SPIFFTX.bit.SPIRST = SPIFFTX_SPIRST_RESUME; // 释放复位 SPI 通道
    SpiaRegs.SPIFFTX.bit.TXFIFO = SPIFFTX_TXFIFO_RELEASE;// 释放复位 TXFIFO
    SpiaRegs.SPIFFTX.bit.SPIFFENA = SPIFFTX_SPIFFENA_ENABLE;// SPI FIFO 使能
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = SPIFFTX_TXFFINTCLR_CLEAR;// 清除 TXFIFO 中断标志
    SpiaRegs.SPIFFTX.bit.TXFFIENA = SPIFFTX_TXFFIENA_DISABLE;// 禁止 TXFIFO 中断
    SpiaRegs.SPIFFTX.bit.TXFFIL = 0;

    SpiaRegs.SPIFFRX.bit.RXFIFORESET = SPIFFRX_RXFIFORESET_HOLDRESET;// 复位 RXFIFO

    SpiaRegs.SPIFFRX.bit.RXFIFORESET = SPIFFRX_RXFIFORESET_RELEASE;// 释放复位
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = SPIFFRX_RXFFOVRCLR_CLEAR;// 清除 RXFIFO 溢出标志
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = SPIFFRX_RXFFINTCLR_CLEAR;// 清除 RXFIFO 中断标志
    SpiaRegs.SPIFFRX.bit.RXFFIENA = SPIFFRX_RXFFIENA_ENABLE;// 禁止 RXFIFO 中断
    SpiaRegs.SPIFFRX.bit.RXFFIL = 0;

    SpiaRegs.SPIFFCT.bit.TXDLY = 0x0;//发送延时控制
}

//
// End of file
//
