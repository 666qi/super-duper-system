//###########################################################################
//
// FILE:    f28004x_dma.c
//
// TITLE:   f28004x DMA Initialization & Support Functions.
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
#include "f28004x_device.h"
#include "f28004x_examples.h"


//extern uint16_t myADCADataBuffer[RESULTS_BUFFER_SIZE]; // Buffer for results
//extern uint16_t myADCCDataBuffer[RESULTS_BUFFER_SIZE]; // Buffer for results

//
// DMAInitialize - This function initializes the DMA to a known state.
//
void DMAInitialize(void)
{
    EALLOW;

    //
    // Perform a hard reset on DMA
    //
    DmaRegs.DMACTRL.bit.HARDRESET = 1; // 重置整个DMA
    __asm (" nop"); // one NOP required after HARDRESET

    //
    // Allow DMA to run free on emulation suspend
    //
    DmaRegs.DEBUGCTRL.bit.FREE = 1; //DMA在模拟停止期间继续运行。

    EDIS;
}

//
// DMACH1AddrConfig - DMA Channel 1 Address Configuration
//
void DMACH1AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;   // Point to
                                                            // beginning of
                                                            // source buffer
    DmaRegs.CH1.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest;  // Point to
                                                         // beginning of
                                                         // destination buffer
    DmaRegs.CH1.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH1BurstConfig - DMA Channel 1 Burst size configuration
//
void DMACH1BurstConfig(Uint16 bsize, int16 srcbstep, int16 desbstep)
{
    EALLOW;

    //
    // Set up BURST registers:
    //
    // BURST_SIZE
    // BURST_SIZE寄存器允许在一个突发中最多传输32个16位的字。
    DmaRegs.CH1.BURST_SIZE.all = bsize;      // Number of words(X-1)
                                             // x-ferred in a burst.
    // 源的步长
    DmaRegs.CH1.SRC_BURST_STEP = srcbstep;   // Increment source addr between
                                             // each word x-ferred.
    // 目的步长
    DmaRegs.CH1.DST_BURST_STEP = desbstep;   // Increment dest addr between
                                             // each word x-ferred.

    EDIS;
}

//
// DMACH1TransferConfig - DMA Channel 1 Transfer size configuration
//
void DMACH1TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep)
{
    EALLOW;

    //
    // Set up TRANSFER registers:
    //
    // TRANSFER_SIZE
    DmaRegs.CH1.TRANSFER_SIZE = tsize;        // Number of bursts per transfer,
                                              // DMA interrupt will occur after
                                              // completed transfer.

    DmaRegs.CH1.SRC_TRANSFER_STEP = srctstep; // TRANSFER_STEP is ignored
                                              // when WRAP occurs.
    DmaRegs.CH1.DST_TRANSFER_STEP = deststep; // TRANSFER_STEP is ignored
                                              // when WRAP occurs.

    EDIS;
}

//
// DMACH1WrapConfig - DMA Channel 1 Wrap size configuration
//
void DMACH1WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize,
                      int16 deswstep)
{
    EALLOW;

    //
    // Set up WRAP registers:
    //
    DmaRegs.CH1.SRC_WRAP_SIZE = srcwsize; // Wrap source address after N bursts
    DmaRegs.CH1.SRC_WRAP_STEP = srcwstep; // Step for source wrap
    DmaRegs.CH1.DST_WRAP_SIZE = deswsize; // Wrap destination address after
                                          // N bursts.
    DmaRegs.CH1.DST_WRAP_STEP = deswstep; // Step for destination wrap

    EDIS;
}

//
// DMACH1ModeConfig - DMA Channel 1 Mode configuration
//
void DMACH1ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot,
                      Uint16 cont, Uint16 synce, Uint16 syncsel,
                      Uint16 ovrinte, Uint16 datasize, Uint16 chintmode,
                      Uint16 chinte)
{
    EALLOW;

    //
    // Set up MODE Register:
    // persel - Source select
    // PERINTSEL - Should be hard coded to channel, above now selects source
    // PERINTE - Peripheral interrupt enable
    // ONESHOT - Oneshot enable
    // CONTINUOUS - Continuous enable
    // OVRINTE - Enable/disable the overflow interrupt
    // DATASIZE - 16-bit/32-bit data size transfers
    // CHINTMODE - Generate interrupt to CPU at beginning/end of transfer
    // CHINTE - Channel Interrupt to  CPU enable
    //
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = persel;  // P_1327 Table 10-1. DMA Trigger Source Options
    DmaRegs.CH1.MODE.bit.PERINTSEL = 1;              // 这些是遗留位，应该设置为通道号。
    DmaRegs.CH1.MODE.bit.PERINTE = perinte;          // 外设事件触发器启用
    DmaRegs.CH1.MODE.bit.ONESHOT = oneshot;          // One Shot Mode
    DmaRegs.CH1.MODE.bit.CONTINUOUS = cont;          // Continuous Mode
    DmaRegs.CH1.MODE.bit.OVRINTE = ovrinte;          // 溢出中断启用
    DmaRegs.CH1.MODE.bit.DATASIZE = datasize;        // 数据大小
    DmaRegs.CH1.MODE.bit.CHINTMODE = chintmode;      // 通道中断生成的模式
    DmaRegs.CH1.MODE.bit.CHINTE = chinte;            // 通道中断启用

    //
    // Clear any spurious flags: interrupt and sync error flags
    //
    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;
    DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;

    //
    // Initialize PIE vector for CPU interrupt:
    // Enable DMA CH1 interrupt in PIE
    //
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;

    EDIS;
}

//
// StartDMACH1 - This function starts DMA Channel 1.
//
void StartDMACH1(void)
{
    EALLOW;
    DmaRegs.CH1.CONTROL.bit.RUN = 1;
    EDIS;
}

//
// DMACH2AddrConfig - DMA Channel 2 Address Configuration
//
void DMACH2AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH2.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;   // Point to
                                                            // beginning of
                                                            // source buffer.
    DmaRegs.CH2.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH2.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest; // Point to beginning
                                                         // of destination
                                                         // buffer.
    DmaRegs.CH2.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH2BurstConfig - DMA Channel 2 Burst size configuration
//
void DMACH2BurstConfig(Uint16 bsize, int16 srcbstep, int16 desbstep)
{
    EALLOW;

    //
    // Set up BURST registers:
    //
    DmaRegs.CH2.BURST_SIZE.all = bsize;     // Number of words(X-1) x-ferred in
                                            // a burst.
    DmaRegs.CH2.SRC_BURST_STEP = srcbstep;  // Increment source addr between
                                            // each word x-ferred.
    DmaRegs.CH2.DST_BURST_STEP = desbstep;  // Increment dest addr between each
                                            // word x-ferred.

    EDIS;
}

//
// DMACH2TransferConfig - DMA Channel 2 Transfer size Configuration
//
void DMACH2TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep)
{
    EALLOW;

    //
    // Set up TRANSFER registers:
    //
    DmaRegs.CH2.TRANSFER_SIZE = tsize;        // Number of bursts per transfer,
                                              // DMA interrupt will occur after
                                              // completed transfer.
    DmaRegs.CH2.SRC_TRANSFER_STEP = srctstep; // TRANSFER_STEP is ignored when
                                              // WRAP occurs.
    DmaRegs.CH2.DST_TRANSFER_STEP = deststep; // TRANSFER_STEP is ignored when
                                              // WRAP occurs.

    EDIS;
}

//
// DMACH2WrapConfig - DMA Channel 2 Wrap size configuration
//
void DMACH2WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize,
                      int16 deswstep)
{
    EALLOW;

    //
    // Set up WRAP registers:
    //
    DmaRegs.CH2.SRC_WRAP_SIZE = srcwsize; // Wrap source address after N bursts
    DmaRegs.CH2.SRC_WRAP_STEP = srcwstep; // Step for source wrap

    DmaRegs.CH2.DST_WRAP_SIZE = deswsize; // Wrap destination address after
                                          // N bursts.
    DmaRegs.CH2.DST_WRAP_STEP = deswstep; // Step for destination wrap

    EDIS;
}

//
// DMACH2ModeConfig - DMA Channel 2 Mode configuration
//
void DMACH2ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot,
                      Uint16 cont, Uint16 synce, Uint16 syncsel,
                      Uint16 ovrinte, Uint16 datasize, Uint16 chintmode,
                      Uint16 chinte)
{
    EALLOW;

    //
    // Set up MODE Register:
    // persel - Source select
    // PERINTSEL - Should be hard coded to channel, above now selects source
    // PERINTE - Peripheral interrupt enable
    // ONESHOT - Oneshot enable
    // CONTINUOUS - Continuous enable
    // OVRINTE - Enable/disable the overflow interrupt
    // DATASIZE - 16-bit/32-bit data size transfers
    // CHINTMODE - Generate interrupt to CPU at beginning/end of transfer
    // CHINTE - Channel Interrupt to  CPU enable
    //
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = persel;
    DmaRegs.CH2.MODE.bit.PERINTSEL = 2;
    DmaRegs.CH2.MODE.bit.PERINTE = perinte;
    DmaRegs.CH2.MODE.bit.ONESHOT = oneshot;
    DmaRegs.CH2.MODE.bit.CONTINUOUS = cont;
    DmaRegs.CH2.MODE.bit.OVRINTE = ovrinte;
    DmaRegs.CH2.MODE.bit.DATASIZE = datasize;
    DmaRegs.CH2.MODE.bit.CHINTMODE = chintmode;
    DmaRegs.CH2.MODE.bit.CHINTE = chinte;

    //
    // Clear any spurious flags: Interrupt flags and sync error flags
    //
    DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1;
    DmaRegs.CH2.CONTROL.bit.ERRCLR = 1;

    //
    // Initialize PIE vector for CPU interrupt:
    // Enable DMA CH2 interrupt in PIE
    //
    PieCtrlRegs.PIEIER7.bit.INTx2 = 1;

    EDIS;
}

//
// StartDMACH2 - This function starts DMA Channel 2.
//
void StartDMACH2(void)
{
    EALLOW;
    DmaRegs.CH2.CONTROL.bit.RUN = 1;
    EDIS;
}

//
// DMACH3AddrConfig - DMA Channel 3 Address configuration
//
void DMACH3AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH3.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;
                                                          // Point to beginning
                                                          // of source buffer.
    DmaRegs.CH3.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH3.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest; // Point to beginning
                                                        // of destination
                                                        // buffer.
    DmaRegs.CH3.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH3BurstConfig - DMA Channel 3 burst size configuration
//
void DMACH3BurstConfig(Uint16 bsize, int16 srcbstep, int16 desbstep)
{
    EALLOW;

    //
    // Set up BURST registers:
    //
    DmaRegs.CH3.BURST_SIZE.all = bsize;     // Number of words(X-1) x-ferred in
                                            // a burst.
    DmaRegs.CH3.SRC_BURST_STEP = srcbstep;  // Increment source addr between
                                            // each word x-ferred.
    DmaRegs.CH3.DST_BURST_STEP = desbstep;  // Increment dest addr between each
                                            // word x-ferred.

    EDIS;
}

//
// DMACH3TransferConfig - DMA channel 3 transfer size configuration
//
void DMACH3TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep)
{
    EALLOW;

    //
    // Set up TRANSFER registers:
    //
    DmaRegs.CH3.TRANSFER_SIZE = tsize;        // Number of bursts per transfer,
                                              // DMA interrupt will occur after
                                              // completed transfer.
    DmaRegs.CH3.SRC_TRANSFER_STEP = srctstep; // TRANSFER_STEP is ignored when
                                              // WRAP occurs.
    DmaRegs.CH3.DST_TRANSFER_STEP = deststep; // TRANSFER_STEP is ignored when
                                              // WRAP occurs.

    EDIS;
}

//
// DMACH3WrapConfig - DMA Channel 3 wrap size configuration
//
void DMACH3WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize,
                      int16 deswstep)
{
    EALLOW;

    //
    // Set up WRAP registers:
    //
    DmaRegs.CH3.SRC_WRAP_SIZE = srcwsize; // Wrap source address after N bursts
    DmaRegs.CH3.SRC_WRAP_STEP = srcwstep; // Step for source wrap

    DmaRegs.CH3.DST_WRAP_SIZE = deswsize; // Wrap destination address after N
                                          // bursts.
    DmaRegs.CH3.DST_WRAP_STEP = deswstep; // Step for destination wrap

    EDIS;
}

//
// DMACH3ModeConfig - DMA Channel 3 mode configuration
//
void DMACH3ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot,
                      Uint16 cont, Uint16 synce, Uint16 syncsel,
                      Uint16 ovrinte, Uint16 datasize, Uint16 chintmode,
                      Uint16 chinte)
{
    EALLOW;

    //
    // Set up MODE Register:
    // persel - Source select
    // PERINTSEL - Should be hard coded to channel, above now selects source
    // PERINTE - Peripheral interrupt enable
    // ONESHOT - Oneshot enable
    // CONTINUOUS - Continuous enable
    // OVRINTE - Enable/disable the overflow interrupt
    // DATASIZE - 16-bit/32-bit data size transfers
    // CHINTMODE - Generate interrupt to CPU at beginning/end of transfer
    // CHINTE - Channel Interrupt to  CPU enable
    //
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH3 = persel;
    DmaRegs.CH3.MODE.bit.PERINTSEL = 3;
    DmaRegs.CH3.MODE.bit.PERINTE = perinte;
    DmaRegs.CH3.MODE.bit.ONESHOT = oneshot;
    DmaRegs.CH3.MODE.bit.CONTINUOUS = cont;
    DmaRegs.CH3.MODE.bit.OVRINTE = ovrinte;
    DmaRegs.CH3.MODE.bit.DATASIZE = datasize;
    DmaRegs.CH3.MODE.bit.CHINTMODE = chintmode;
    DmaRegs.CH3.MODE.bit.CHINTE = chinte;

    //
    // Clear any spurious flags: interrupt flags and sync error flags
    //
    DmaRegs.CH3.CONTROL.bit.PERINTCLR = 1;
    DmaRegs.CH3.CONTROL.bit.ERRCLR = 1;

    //
    // Initialize PIE vector for CPU interrupt:
    // Enable DMA CH3 interrupt in PIE
    //
    PieCtrlRegs.PIEIER7.bit.INTx3 = 1;

    EDIS;
}

//
// StartDMACH3 - This function starts DMA Channel 3.
//
void StartDMACH3(void)
{
    EALLOW;
    DmaRegs.CH3.CONTROL.bit.RUN = 1;
    EDIS;
}

//
// DMACH4AddrConfig - DMA Channel 4 address configuration
//
void DMACH4AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH4.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;
                                                          // Point to beginning
                                                          // of source buffer.
    DmaRegs.CH4.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH4.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest; // Point to beginning
                                                          // of destination
                                                          // buffer.
    DmaRegs.CH4.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH4BurstConfig - DMA Channel 4 burst size configuration
//
void DMACH4BurstConfig(Uint16 bsize, int16 srcbstep, int16 desbstep)
{
    EALLOW;

    //
    // Set up BURST registers:
    //
    DmaRegs.CH4.BURST_SIZE.all = bsize;     // Number of words(X-1) x-ferred in
                                            // a burst.
    DmaRegs.CH4.SRC_BURST_STEP = srcbstep;  // Increment source addr between
                                            // each word x-ferred.
    DmaRegs.CH4.DST_BURST_STEP = desbstep;  // Increment dest addr between each
                                            // word x-ferred.

    EDIS;
}

//
// DMACH4TransferConfig - DMA channel 4 transfer size configuration
//
void DMACH4TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep)
{
    EALLOW;

    //
    // Set up TRANSFER registers:
    //
    DmaRegs.CH4.TRANSFER_SIZE = tsize;        // Number of bursts per transfer,
                                              // DMA interrupt will occur after
                                              // completed transfer.
    DmaRegs.CH4.SRC_TRANSFER_STEP = srctstep; // TRANSFER_STEP is ignored when
                                              // WRAP occurs.
    DmaRegs.CH4.DST_TRANSFER_STEP = deststep; // TRANSFER_STEP is ignored when
                                              // WRAP occurs.

    EDIS;
}

//
// DMACH4WrapConfig - DMA channel 4 wrap size configuration
//
void DMACH4WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize,
                      int16 deswstep)
{
    EALLOW;

    //
    // Set up WRAP registers:
    //
    DmaRegs.CH4.SRC_WRAP_SIZE = srcwsize; // Wrap source address after N bursts
    DmaRegs.CH4.SRC_WRAP_STEP = srcwstep; // Step for source wrap

    DmaRegs.CH4.DST_WRAP_SIZE = deswsize; // Wrap destination address after
                                          // N bursts.
    DmaRegs.CH4.DST_WRAP_STEP = deswstep; // Step for destination wrap

    EDIS;
}

//
// DMACH4ModeConfig - DMA Channel 4 mode configuration
//
void DMACH4ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot,
                      Uint16 cont, Uint16 synce, Uint16 syncsel,
                      Uint16 ovrinte, Uint16 datasize, Uint16 chintmode,
                      Uint16 chinte)
{
    EALLOW;

    //
    // Set up MODE Register:
    // persel - Source select
    // PERINTSEL - Should be hard coded to channel, above now selects source
    // PERINTE - Peripheral interrupt enable
    // ONESHOT - Oneshot enable
    // CONTINUOUS - Continuous enable
    // OVRINTE - Enable/disable the overflow interrupt
    // DATASIZE - 16-bit/32-bit data size transfers
    // CHINTMODE - Generate interrupt to CPU at beginning/end of transfer
    // CHINTE - Channel Interrupt to  CPU enable
    //
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH4 = persel;
    DmaRegs.CH4.MODE.bit.PERINTSEL = 4;
    DmaRegs.CH4.MODE.bit.PERINTE = perinte;
    DmaRegs.CH4.MODE.bit.ONESHOT = oneshot;
    DmaRegs.CH4.MODE.bit.CONTINUOUS = cont;
    DmaRegs.CH4.MODE.bit.OVRINTE = ovrinte;
    DmaRegs.CH4.MODE.bit.DATASIZE = datasize;
    DmaRegs.CH4.MODE.bit.CHINTMODE = chintmode;
    DmaRegs.CH4.MODE.bit.CHINTE = chinte;

    //
    // Clear any spurious flags: Interrupt flags and sync error flags
    //
    DmaRegs.CH4.CONTROL.bit.PERINTCLR = 1;
    DmaRegs.CH4.CONTROL.bit.ERRCLR = 1;

    //
    // Initialize PIE vector for CPU interrupt:
    // Enable DMA CH4 interrupt in PIE
    //
    PieCtrlRegs.PIEIER7.bit.INTx4 = 1;

    EDIS;
}

//
// StartDMACH4 - This function starts DMA Channel 4.
//
void StartDMACH4(void)
{
    EALLOW;
    DmaRegs.CH4.CONTROL.bit.RUN = 1;
    EDIS;
}

//
// DMACH5AddrConfig - DMA channel 5 address configuration
//
void DMACH5AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH5.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;
                                                          // Point to beginning
                                                          // of source buffer
    DmaRegs.CH5.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH5.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest; // Point to beginning
                                                         // of destination
                                                         // buffer.
    DmaRegs.CH5.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH5BurstConfig - DMA Channel 5 burst size configuration
//
void DMACH5BurstConfig(Uint16 bsize, int16 srcbstep, int16 desbstep)
{
    EALLOW;

    //
    // Set up BURST registers:
    //
    DmaRegs.CH5.BURST_SIZE.all = bsize;     // Number of words(X-1) x-ferred in
                                            // a burst.
    DmaRegs.CH5.SRC_BURST_STEP = srcbstep;  // Increment source addr between
                                            // each word x-ferred.
    DmaRegs.CH5.DST_BURST_STEP = desbstep;  // Increment dest addr between each
                                            // word x-ferred.

    EDIS;
}

//
// DMACH5TransferConfig - DMA channel 5 transfer size configuration
//
void DMACH5TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep)
{
    EALLOW;

    //
    // Set up TRANSFER registers:
    //
    DmaRegs.CH5.TRANSFER_SIZE = tsize;        // Number of bursts per transfer,
                                              // DMA interrupt will occur after
                                              // completed transfer.
    DmaRegs.CH5.SRC_TRANSFER_STEP = srctstep; // TRANSFER_STEP is ignored when
                                              // WRAP occurs.
    DmaRegs.CH5.DST_TRANSFER_STEP = deststep; // TRANSFER_STEP is ignored when
                                              // WRAP occurs.

    EDIS;
}

//
// DMACH5WrapConfig - DMA Channel 5 wrap size configuration
//
void DMACH5WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize,
                      int16 deswstep)
{
    EALLOW;

    //
    // Set up WRAP registers:
    //
    DmaRegs.CH5.SRC_WRAP_SIZE = srcwsize; // Wrap source address after N bursts
    DmaRegs.CH5.SRC_WRAP_STEP = srcwstep; // Step for source wrap

    DmaRegs.CH5.DST_WRAP_SIZE = deswsize; // Wrap destination address after
                                          // N bursts.
    DmaRegs.CH5.DST_WRAP_STEP = deswstep; // Step for destination wrap

    EDIS;
}

//
// DMACH5ModeConfig - DMA Channel 5 mode configuration
//
void DMACH5ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot,
                      Uint16 cont, Uint16 synce, Uint16 syncsel,
                      Uint16 ovrinte, Uint16 datasize, Uint16 chintmode,
                      Uint16 chinte)
{
    EALLOW;

    //
    // Set up MODE Register:
    // persel - Source select
    // PERINTSEL - Should be hard coded to channel, above now selects source
    // PERINTE - Peripheral interrupt enable
    // ONESHOT - Oneshot enable
    // CONTINUOUS - Continuous enable
    // OVRINTE - Enable/disable the overflow interrupt
    // DATASIZE - 16-bit/32-bit data size transfers
    // CHINTMODE - Generate interrupt to CPU at beginning/end of transfer
    // CHINTE - Channel Interrupt to  CPU enable
    //
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH5 = persel;
    DmaRegs.CH5.MODE.bit.PERINTSEL = 5;
    DmaRegs.CH5.MODE.bit.PERINTE = perinte;
    DmaRegs.CH5.MODE.bit.ONESHOT = oneshot;
    DmaRegs.CH5.MODE.bit.CONTINUOUS = cont;
    DmaRegs.CH5.MODE.bit.OVRINTE = ovrinte;
    DmaRegs.CH5.MODE.bit.DATASIZE = datasize;
    DmaRegs.CH5.MODE.bit.CHINTMODE = chintmode;
    DmaRegs.CH5.MODE.bit.CHINTE = chinte;

    //
    // Clear any spurious flags: Interrupt flags and sync error flags
    //
    DmaRegs.CH5.CONTROL.bit.PERINTCLR = 1;
    DmaRegs.CH5.CONTROL.bit.ERRCLR = 1;

    //
    // Initialize PIE vector for CPU interrupt:
    // Enable DMA CH5 interrupt in PIE
    //
    PieCtrlRegs.PIEIER7.bit.INTx5 = 1;

    EDIS;
}

//
// StartDMACH5 - This function starts DMA Channel 5.
//
void StartDMACH5(void)
{
    EALLOW;
    DmaRegs.CH5.CONTROL.bit.RUN = 1;
    EDIS;
}

//
// DMACH6AddrConfig - DMA Channel 6 address configuration
//
void DMACH6AddrConfig(volatile Uint16 *DMA_Dest,volatile Uint16 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH6.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;
                                                          // Point to beginning
                                                          // of source buffer.
    DmaRegs.CH6.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH6.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest; // Point to beginning
                                                         // of destination
                                                         // buffer.
    DmaRegs.CH6.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH6BurstConfig - DMA Channel 6 burst size configuration
//
void DMACH6BurstConfig(Uint16 bsize,Uint16 srcbstep, int16 desbstep)
{
    EALLOW;

    //
    // Set up BURST registers:
    //
    DmaRegs.CH6.BURST_SIZE.all = bsize;     // Number of words(X-1) x-ferred in
                                            // a burst.
    DmaRegs.CH6.SRC_BURST_STEP = srcbstep;  // Increment source addr between
                                            // each word x-ferred.
    DmaRegs.CH6.DST_BURST_STEP = desbstep;  // Increment dest addr between each
                                            // word x-ferred.

    EDIS;
}

//
// DMACH6TransferConfig - DMA channel 6 transfer size configuration
//
void DMACH6TransferConfig(Uint16 tsize, int16 srctstep, int16 deststep)
{
    EALLOW;

    //
    // Set up TRANSFER registers:
    //
    DmaRegs.CH6.TRANSFER_SIZE = tsize;        // Number of bursts per transfer,
                                              // DMA interrupt will occur after
                                              // completed transfer.
    DmaRegs.CH6.SRC_TRANSFER_STEP = srctstep; // TRANSFER_STEP is ignored when
                                              // WRAP occurs.
    DmaRegs.CH6.DST_TRANSFER_STEP = deststep; // TRANSFER_STEP is ignored when
                                              // WRAP occurs.

    EDIS;
}

//
// DMACH6WrapConfig - DMA Channel 6 wrap size configuration
//
void DMACH6WrapConfig(Uint16 srcwsize, int16 srcwstep, Uint16 deswsize,
                      int16 deswstep)
{
    EALLOW;

    //
    // Set up WRAP registers:
    //
    DmaRegs.CH6.SRC_WRAP_SIZE = srcwsize; // Wrap source address after N bursts
    DmaRegs.CH6.SRC_WRAP_STEP = srcwstep; // Step for source wrap

    DmaRegs.CH6.DST_WRAP_SIZE = deswsize; // Wrap destination address after N
                                          // bursts.
    DmaRegs.CH6.DST_WRAP_STEP = deswstep; // Step for destination wrap

    EDIS;
}

//
// DMACH6ModeConfig - DMA Channel 6 mode configuration
//
void DMACH6ModeConfig(Uint16 persel, Uint16 perinte, Uint16 oneshot,
                      Uint16 cont, Uint16 synce, Uint16 syncsel,
                      Uint16 ovrinte, Uint16 datasize, Uint16 chintmode,
                      Uint16 chinte)
{
    EALLOW;

    //
    // Set up MODE Register:
    // persel - Source select
    // PERINTSEL - Should be hard coded to channel, above now selects source
    // PERINTE - Peripheral interrupt enable
    // ONESHOT - Oneshot enable
    // CONTINUOUS - Continuous enable
    // OVRINTE - Enable/disable the overflow interrupt
    // DATASIZE - 16-bit/32-bit data size transfers
    // CHINTMODE - Generate interrupt to CPU at beginning/end of transfer
    // CHINTE - Channel Interrupt to  CPU enable
    //
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH6 = persel;
    DmaRegs.CH6.MODE.bit.PERINTSEL = 6;
    DmaRegs.CH6.MODE.bit.PERINTE = perinte;
    DmaRegs.CH6.MODE.bit.ONESHOT = oneshot;
    DmaRegs.CH6.MODE.bit.CONTINUOUS = cont;
    DmaRegs.CH6.MODE.bit.OVRINTE = ovrinte;
    DmaRegs.CH6.MODE.bit.DATASIZE = datasize;
    DmaRegs.CH6.MODE.bit.CHINTMODE = chintmode;
    DmaRegs.CH6.MODE.bit.CHINTE = chinte;

    //
    // Clear any spurious flags: Interrupt flags and sync error flags
    //
    DmaRegs.CH6.CONTROL.bit.PERINTCLR = 1;
    DmaRegs.CH6.CONTROL.bit.ERRCLR = 1;

    //
    // Initialize PIE vector for CPU interrupt:
    // Enable DMA CH6 interrupt in PIE
    //
    PieCtrlRegs.PIEIER7.bit.INTx6 = 1;

    EDIS;
}

//
// StartDMACH6 - This function starts DMA Channel 6.
//
void StartDMACH6(void)
{
    EALLOW;
    DmaRegs.CH6.CONTROL.bit.RUN = 1;
    EDIS;
}

//
// DMACH1AddrConfig32bit - DMA Channel 1 address configuration for 32bit
//
void DMACH1AddrConfig32bit(volatile Uint32 *DMA_Dest,
                           volatile Uint32 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;
                                                          // Point to beginning
                                                          // of source buffer
    DmaRegs.CH1.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest; // Point to beginning
                                                         // of destination
                                                         // buffer
    DmaRegs.CH1.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH2AddrConfig32bit - DMA Channel 2 address configuration for 32bit
//
void DMACH2AddrConfig32bit(volatile Uint32 *DMA_Dest,
                           volatile Uint32 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH2.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;
                                                          // Point to beginning
                                                          // of source buffer
    DmaRegs.CH2.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH2.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest; // Point to beginning
                                                         // of destination
                                                         // buffer
    DmaRegs.CH2.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH3AddrConfig32bit - DMA Channel 3 address configuration for 32bit
//
void DMACH3AddrConfig32bit(volatile Uint32 *DMA_Dest,
                           volatile Uint32 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH3.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;
                                                          // Point to beginning
                                                          // of source buffer
    DmaRegs.CH3.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH3.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest; // Point to beginning
                                                         // of destination
                                                         // buffer.
    DmaRegs.CH3.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH4AddrConfig32bit - DMA Channel 4 address configuration for 32bit
//
void DMACH4AddrConfig32bit(volatile Uint32 *DMA_Dest,
                           volatile Uint32 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH4.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;
                                                          // Point to beginning
                                                          // of source buffer
    DmaRegs.CH4.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH4.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest;
                                                          // Point to beginning
                                                          // of destination
                                                          // buffer
    DmaRegs.CH4.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH5AddrConfig32bit - DMA Channel 5 address configuration for 32bit
//
void DMACH5AddrConfig32bit(volatile Uint32 *DMA_Dest,
                           volatile Uint32 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH5.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;
                                                          // Point to beginning
                                                          // of source buffer
    DmaRegs.CH5.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH5.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest;
                                                          // Point to beginning
                                                          // of destination
                                                          // buffer
    DmaRegs.CH5.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}

//
// DMACH6AddrConfig32bit - DMA Channel 6 address configuration for 32bit
//
void DMACH6AddrConfig32bit(volatile Uint32 *DMA_Dest,
                           volatile Uint32 *DMA_Source)
{
    EALLOW;

    //
    // Set up SOURCE address:
    //
    DmaRegs.CH6.SRC_BEG_ADDR_SHADOW = (Uint32)DMA_Source;
                                                          // Point to beginning
                                                          // of source buffer
    DmaRegs.CH6.SRC_ADDR_SHADOW =     (Uint32)DMA_Source;

    //
    // Set up DESTINATION address:
    //
    DmaRegs.CH6.DST_BEG_ADDR_SHADOW = (Uint32)DMA_Dest; // Point to beginning
                                                          // of destination
                                                          // buffer
    DmaRegs.CH6.DST_ADDR_SHADOW =     (Uint32)DMA_Dest;

    EDIS;
}


// 自己添加的函数
/* --------------------------------------------------------------
 * 用户根据自己的实际需求修改
 * --------------------------------------------------------------*/

// -------------------------------------------------------------------------
/*
 * 函数功能 ：配置实际的 DMA_CH1
 * 配置步骤：
 * 1、配置 DMA 的源地址和目的地址
 * 2、配置 Burst 模式下传输的 SIZE 源步长和目的步长 （内循环）
 * 3、配置 Transfer 传输的 SIZE 源步长和目的步长    （外循环）
 * 4、配置传输的 Mode
 */
// -------------------------------------------------------------------------
//void ConfigureDMA_CH1(void){
//
//    // 源地址 AdcaResultRegs 目的地址 myADCADataBuffer
//    // ---------------------------------------------------------------------------
//    DMACH1AddrConfig((uint16_t *)&myADCADataBuffer,(uint16_t *)&AdcaResultRegs);
//
//    // BURST_SIZE 16
//    // SRC_BURST_STEP 2
//    // DST_BURST_STEP 2
//    // 每次 BURST 传送 16 个字 ，传送完毕后，源地址 +2 ，目的地址 +2
//    // 详见 P_1330 10.3 Address Pointer and Transfer Control
//    // ---------------------------------------------------------------------------
//    DMACH1BurstConfig(16, 2, 2);
//
//    // TRANSFER_SIZE RESULTS_BUFFER_SIZE >> 4  = 1024/16
//    // SRC_TRANSFER_STEP -14
//    // DST_TRANSFER_STEP 2
//    // 详见 P_1330 10.3 Address Pointer and Transfer Control
//    // ---------------------------------------------------------------------------
//    DMACH1TransferConfig((RESULTS_BUFFER_SIZE >> 4), -16, 0);
//
//    // DMA_ADCAINT2                             详见 P_1327 Table 10-1. DMA Trigger Source Options
//    // PERINT_ENABLE    外设事件触发器启用       详见 P_1346 Figure 10-11. MODE Register
//    // ONESHOT_DISABLE  One Shot Mode 禁用      详见 P_1346 Figure 10-11. MODE Register
//    // CONT_DISABLE     Continuous Mode 禁用    详见 P_1346 Figure 10-11. MODE Register
//    // SYNC_DISABLE     忽略
//    // SYNC_SRC         忽略
//    // OVRFLOW_DISABLE  溢出中断禁用            详见 P_1346 Figure 10-11. MODE Register
//    // THIRTYTWO_BIT    数据大小 32-bit         详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_END        传输完毕后通道中断生成   详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_ENABLE     中断使能                详见 P_1346 Figure 10-11. MODE Register
//    // ---------------------------------------------------------------------------
//    DMACH1ModeConfig( DMA_ADCAINT2, PERINT_ENABLE, ONESHOT_DISABLE, CONT_DISABLE, SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, THIRTYTWO_BIT, CHINT_END,CHINT_ENABLE);
//
//    DmaRegs.CH1.CONTROL.bit.PERINTCLR = DMA_CONTROL_PERINTCLR; // 清除 CH_1 的外设中断标志 PERINTFLG
//}
//
//// -------------------------------------------------------------------------
///*
// * 函数功能 ：配置实际的 DMA_CH2
// * 配置步骤：
// * 1、配置 DMA 的源地址和目的地址
// * 2、配置 Burst 模式下传输的 SIZE 源步长和目的步长 （内循环）
// * 3、配置 Transfer 传输的 SIZE 源步长和目的步长    （外循环）
// * 4、配置传输的 Mode
// */
//// -------------------------------------------------------------------------
//void ConfigureDMA_CH2(void){
//
//    // 源地址 AdcaResultRegs 目的地址 myADCADataBuffer
//    // ---------------------------------------------------------------------------
//    DMACH2AddrConfig((uint16_t *)&myADCADataBuffer,(uint16_t *)&AdcaResultRegs);
//
//    // BURST_SIZE 16
//    // SRC_BURST_STEP 2
//    // DST_BURST_STEP 2
//    // 每次 BURST 传送 16 个字 ，传送完毕后，源地址 +2 ，目的地址 +2
//    // ---------------------------------------------------------------------------
//    DMACH2BurstConfig(16, 2, 2);
//
//    // TRANSFER_SIZE RESULTS_BUFFER_SIZE >> 4  = 1024/16
//    // SRC_TRANSFER_STEP -14
//    // DST_TRANSFER_STEP 2
//    // ---------------------------------------------------------------------------
//    DMACH2TransferConfig((RESULTS_BUFFER_SIZE >> 4), -14, 2);
//
//    // DMA_ADCAINT2                             详见 P_1327 Table 10-1. DMA Trigger Source Options
//    // PERINT_ENABLE    外设事件触发器启用       详见 P_1346 Figure 10-11. MODE Register
//    // ONESHOT_DISABLE  One Shot Mode 禁用      详见 P_1346 Figure 10-11. MODE Register
//    // CONT_DISABLE     Continuous Mode 禁用    详见 P_1346 Figure 10-11. MODE Register
//    // SYNC_DISABLE     忽略
//    // SYNC_SRC         忽略
//    // OVRFLOW_DISABLE  溢出中断禁用            详见 P_1346 Figure 10-11. MODE Register
//    // THIRTYTWO_BIT    数据大小 32-bit         详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_END        传输完毕后通道中断生成   详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_ENABLE     中断使能                详见 P_1346 Figure 10-11. MODE Register
//    // ---------------------------------------------------------------------------
//    DMACH2ModeConfig( DMA_ADCAINT2, PERINT_ENABLE, ONESHOT_DISABLE, CONT_DISABLE, SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, THIRTYTWO_BIT, CHINT_END,CHINT_ENABLE);
//
//    DmaRegs.CH2.CONTROL.bit.PERINTCLR = DMA_CONTROL_PERINTCLR; // 清除 PERINTFLG
//}
//// -------------------------------------------------------------------------
///*
// * 函数功能 ：配置实际的 DMA_CH3
// * 配置步骤：
// * 1、配置 DMA 的源地址和目的地址
// * 2、配置 Burst 模式下传输的 SIZE 源步长和目的步长 （内循环）
// * 3、配置 Transfer 传输的 SIZE 源步长和目的步长    （外循环）
// * 4、配置传输的 Mode
// */
//// -------------------------------------------------------------------------
//void ConfigureDMA_CH3(void){
//
//    // 源地址 AdcaResultRegs 目的地址 myADCADataBuffer
//    // ---------------------------------------------------------------------------
//    DMACH3AddrConfig((uint16_t *)&myADCADataBuffer,(uint16_t *)&AdcaResultRegs);
//
//    // BURST_SIZE 16
//    // SRC_BURST_STEP 2
//    // DST_BURST_STEP 2
//    // 每次 BURST 传送 16 个字 ，传送完毕后，源地址 +2 ，目的地址 +2
//    // ---------------------------------------------------------------------------
//    DMACH3BurstConfig(16, 2, 2);
//
//    // TRANSFER_SIZE RESULTS_BUFFER_SIZE >> 4  = 1024/16
//    // SRC_TRANSFER_STEP -14
//    // DST_TRANSFER_STEP 2
//    // ---------------------------------------------------------------------------
//    DMACH3TransferConfig((RESULTS_BUFFER_SIZE >> 4), -14, 2);
//
//    // DMA_ADCAINT2                             详见 P_1327 Table 10-1. DMA Trigger Source Options
//    // PERINT_ENABLE    外设事件触发器启用       详见 P_1346 Figure 10-11. MODE Register
//    // ONESHOT_DISABLE  One Shot Mode 禁用      详见 P_1346 Figure 10-11. MODE Register
//    // CONT_DISABLE     Continuous Mode 禁用    详见 P_1346 Figure 10-11. MODE Register
//    // SYNC_DISABLE     忽略
//    // SYNC_SRC         忽略
//    // OVRFLOW_DISABLE  溢出中断禁用            详见 P_1346 Figure 10-11. MODE Register
//    // THIRTYTWO_BIT    数据大小 32-bit         详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_END        传输完毕后通道中断生成   详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_ENABLE     中断使能                详见 P_1346 Figure 10-11. MODE Register
//    // ---------------------------------------------------------------------------
//    DMACH3ModeConfig( DMA_ADCAINT2, PERINT_ENABLE, ONESHOT_DISABLE, CONT_DISABLE, SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, THIRTYTWO_BIT, CHINT_END,CHINT_ENABLE);
//
//}
//// -------------------------------------------------------------------------
///*
// * 函数功能 ：配置实际的 DMA_CH4
// * 配置步骤：
// * 1、配置 DMA 的源地址和目的地址
// * 2、配置 Burst 模式下传输的 SIZE 源步长和目的步长 （内循环）
// * 3、配置 Transfer 传输的 SIZE 源步长和目的步长    （外循环）
// * 4、配置传输的 Mode
// */
//// -------------------------------------------------------------------------
//void ConfigureDMA_CH4(void){
//
//    // 源地址 AdcaResultRegs 目的地址 myADCADataBuffer
//    // ---------------------------------------------------------------------------
//    DMACH4AddrConfig((uint16_t *)&myADCADataBuffer,(uint16_t *)&AdcaResultRegs);
//
//    // BURST_SIZE 16
//    // SRC_BURST_STEP 2
//    // DST_BURST_STEP 2
//    // 每次 BURST 传送 16 个字 ，传送完毕后，源地址 +2 ，目的地址 +2
//    // ---------------------------------------------------------------------------
//    DMACH4BurstConfig(16, 2, 2);
//
//    // TRANSFER_SIZE RESULTS_BUFFER_SIZE >> 4  = 1024/16
//    // SRC_TRANSFER_STEP -14
//    // DST_TRANSFER_STEP 2
//    // ---------------------------------------------------------------------------
//    DMACH4TransferConfig((RESULTS_BUFFER_SIZE >> 4), -14, 2);
//
//    // DMA_ADCAINT2                             详见 P_1327 Table 10-1. DMA Trigger Source Options
//    // PERINT_ENABLE    外设事件触发器启用       详见 P_1346 Figure 10-11. MODE Register
//    // ONESHOT_DISABLE  One Shot Mode 禁用      详见 P_1346 Figure 10-11. MODE Register
//    // CONT_DISABLE     Continuous Mode 禁用    详见 P_1346 Figure 10-11. MODE Register
//    // SYNC_DISABLE     忽略
//    // SYNC_SRC         忽略
//    // OVRFLOW_DISABLE  溢出中断禁用            详见 P_1346 Figure 10-11. MODE Register
//    // THIRTYTWO_BIT    数据大小 32-bit         详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_END        传输完毕后通道中断生成   详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_ENABLE     中断使能                详见 P_1346 Figure 10-11. MODE Register
//    // ---------------------------------------------------------------------------
//    DMACH4ModeConfig( DMA_ADCAINT2, PERINT_ENABLE, ONESHOT_DISABLE, CONT_DISABLE, SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, THIRTYTWO_BIT, CHINT_END,CHINT_ENABLE);
//
//}
//// -------------------------------------------------------------------------
///*
// * 函数功能 ：配置实际的 DMA_CH5
// * 配置步骤：
// * 1、配置 DMA 的源地址和目的地址
// * 2、配置 Burst 模式下传输的 SIZE 源步长和目的步长 （内循环）
// * 3、配置 Transfer 传输的 SIZE 源步长和目的步长    （外循环）
// * 4、配置传输的 Mode
// */
//// -------------------------------------------------------------------------
//void ConfigureDMA_CH5(void){
//
//    // 源地址 AdcaResultRegs 目的地址 myADCADataBuffer
//    // ---------------------------------------------------------------------------
//    DMACH5AddrConfig((uint16_t *)&myADCADataBuffer,(uint16_t *)&AdcaResultRegs);
//
//    // BURST_SIZE 16
//    // SRC_BURST_STEP 2
//    // DST_BURST_STEP 2
//    // 每次 BURST 传送 16 个字 ，传送完毕后，源地址 +2 ，目的地址 +2
//    // ---------------------------------------------------------------------------
//    DMACH5BurstConfig(16, 2, 2);
//
//    // TRANSFER_SIZE RESULTS_BUFFER_SIZE >> 4  = 1024/16
//    // SRC_TRANSFER_STEP -14
//    // DST_TRANSFER_STEP 2
//    // ---------------------------------------------------------------------------
//    DMACH5TransferConfig((RESULTS_BUFFER_SIZE >> 4), -14, 2);
//
//    // DMA_ADCAINT2                             详见 P_1327 Table 10-1. DMA Trigger Source Options
//    // PERINT_ENABLE    外设事件触发器启用       详见 P_1346 Figure 10-11. MODE Register
//    // ONESHOT_DISABLE  One Shot Mode 禁用      详见 P_1346 Figure 10-11. MODE Register
//    // CONT_DISABLE     Continuous Mode 禁用    详见 P_1346 Figure 10-11. MODE Register
//    // SYNC_DISABLE     忽略
//    // SYNC_SRC         忽略
//    // OVRFLOW_DISABLE  溢出中断禁用            详见 P_1346 Figure 10-11. MODE Register
//    // THIRTYTWO_BIT    数据大小 32-bit         详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_END        传输完毕后通道中断生成   详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_ENABLE     中断使能                详见 P_1346 Figure 10-11. MODE Register
//    // ---------------------------------------------------------------------------
//    DMACH5ModeConfig( DMA_ADCAINT2, PERINT_ENABLE, ONESHOT_DISABLE, CONT_DISABLE, SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, THIRTYTWO_BIT, CHINT_END,CHINT_ENABLE);
//
//}
//
//// -------------------------------------------------------------------------
///*
// * 函数功能 ：配置实际的 DMA_CH6
// * 配置步骤：
// * 1、配置 DMA 的源地址和目的地址
// * 2、配置 Burst 模式下传输的 SIZE 源步长和目的步长 （内循环）
// * 3、配置 Transfer 传输的 SIZE 源步长和目的步长    （外循环）
// * 4、配置传输的 Mode
// */
//// -------------------------------------------------------------------------
//void ConfigureDMA_CH6(void){
//
//    // 源地址 AdcaResultRegs 目的地址 myADCADataBuffer
//    // ---------------------------------------------------------------------------
//    DMACH6AddrConfig((uint16_t *)&myADCADataBuffer,(uint16_t *)&AdcaResultRegs);
//
//    // BURST_SIZE 16
//    // SRC_BURST_STEP 2
//    // DST_BURST_STEP 2
//    // 每次 BURST 传送 16 个字 ，传送完毕后，源地址 +2 ，目的地址 +2
//    // ---------------------------------------------------------------------------
//    DMACH6BurstConfig(16, 2, 2);
//
//    // TRANSFER_SIZE RESULTS_BUFFER_SIZE >> 4  = 1024/16
//    // SRC_TRANSFER_STEP -14
//    // DST_TRANSFER_STEP 2
//    // ---------------------------------------------------------------------------
//    DMACH6TransferConfig((RESULTS_BUFFER_SIZE >> 4), -14, 2);
//
//    // DMA_ADCAINT2                             详见 P_1327 Table 10-1. DMA Trigger Source Options
//    // PERINT_ENABLE    外设事件触发器启用       详见 P_1346 Figure 10-11. MODE Register
//    // ONESHOT_DISABLE  One Shot Mode 禁用      详见 P_1346 Figure 10-11. MODE Register
//    // CONT_DISABLE     Continuous Mode 禁用    详见 P_1346 Figure 10-11. MODE Register
//    // SYNC_DISABLE     忽略
//    // SYNC_SRC         忽略
//    // OVRFLOW_DISABLE  溢出中断禁用            详见 P_1346 Figure 10-11. MODE Register
//    // THIRTYTWO_BIT    数据大小 32-bit         详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_END        传输完毕后通道中断生成   详见 P_1346 Figure 10-11. MODE Register
//    // CHINT_ENABLE     中断使能                详见 P_1346 Figure 10-11. MODE Register
//    // ---------------------------------------------------------------------------
//    DMACH6ModeConfig( DMA_ADCAINT2, PERINT_ENABLE, ONESHOT_DISABLE, CONT_DISABLE, SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, THIRTYTWO_BIT, CHINT_END,CHINT_ENABLE);
//
//}
//
// End of file
//
