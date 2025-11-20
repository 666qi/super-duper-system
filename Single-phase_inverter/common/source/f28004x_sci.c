/*
 * f28004x_sci.c
 *
 *  Created on: 2022年6月1日
 *      Author: jzyli
 */

//
// Included Files
//
#include "f28004x_device.h"      // Header File Include File
#include "f28004x_examples.h"    // Examples Include File



// 自己添加的函数
/* --------------------------------------------------------------
 * 用户根据自己的实际需求修改
 * --------------------------------------------------------------*/

/*
 * 函数功能
 * 配置 SCI 功能的 GPIO
 * ----------------------------------------------------------
 */
void Init_SCIAGPIO(void){
    EALLOW;
    //
    // GPIO28 is the SCI Rx pin.
    // SCIA_RX
    // ------------------------------------------------------------------------------------
    GpioCtrlRegs.GPACSEL4.bit.GPIO28 = GPIO_MUX_CPU1; // CPU is the master

    GpioCtrlRegs.GPAGMUX2.bit.GPIO28 = 0;               // 选择 0 组
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;                // 选择 1 功能为 SCIA_RX(SCIA接收)

    GpioCtrlRegs.GPADIR.bit.GPIO28 = GPIO_INPUT;        // GPIO28 = input
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = PUD_DISABLE;      // Disable pull-up on GPIO28
    GpioCtrlRegs.GPAINV.bit.GPIO28 = GPIO_INV_DISABLE; // 禁止输入反极

    // GPACTRL Register
    // GPIO A Qualification Sampling Period
    /*
     * 0x00: Period = 0 SYSCLK cycles
     * 0x01: Period = 2 SYSCLK cycles
     * 0x02: Period = 4 SYSCLK cycles
     * ...
     * 0xFF: Period = 510 SYSCLK cycles
     */
    GpioCtrlRegs.GPACTRL.bit.QUALPRD2 = 0;   // 设置默认采集样本时钟

    // GPAQSEL0 - PAQSEL1
    // 0: Synchronous
    // 1: 3-sample qualification
    // 2: 6-sample qualification
    // 3: Asynchronous
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = GPIO_QSEL_Asynchronous; // 设置异步
    // ------------------------------------------------------------------------------------


    //
    // GPIO29 is the SCI Tx pin.
    // SCIA_TX
    // ------------------------------------------------------------------------------------
    GpioCtrlRegs.GPACSEL4.bit.GPIO29 = GPIO_MUX_CPU1; // CPU is the master

    GpioCtrlRegs.GPAGMUX2.bit.GPIO29 = 0;               // 选择 0 组
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;                // 选择 1 功能为 SCIA_TX(SCIA传输)

    GpioCtrlRegs.GPADIR.bit.GPIO29 = GPIO_OUTPUT;        // GPIO29 = output
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = GPIO_QSEL_Asynchronous; // 设置异步
    EDIS;
}

/* *******************************************************
 *
 * 初始化 SCIA FIFO
 *
 * *******************************************************/
/*
 * 初始化 SCIA FIFO 过程如下
 * 1、复位 SCI 发送/接收 的FIFO通道
 * 2、清空重置 发送/接收 的FIFO数据
 * 3、恢复 SCI FIFO 传输/接收
 * 4、使能 SCI FIFO
 * 5、清除 发送/接收 SCI FIFO中断标志
 * 6、禁止 发送/接收 SCI FIFO中断
 * 7、设置 发送/接收 SCI FIFO中断级数
 * 8、清除 发送/接收 SCI FIFO 自动波特检测完成标志
 */
void Init_SCIA_FIFO(void){

    SciaRegs.SCIFFTX.bit.SCIRST = SCIFFTX_SCIRST_RESET;// 复位 SCI
    SciaRegs.SCIFFTX.bit.TXFIFORESET = SCIFFTX_TXFIFORESET_HOLDRESET;// 复位 FIFO 发送
    SciaRegs.SCIFFRX.bit.RXFIFORESET = SCIFFRX_RXFIFORESET_HOLDRESET;// 复位 FIFO 接收
    DELAY_US(1);
    SciaRegs.SCIFFTX.bit.TXFIFORESET = SCIFFTX_TXFIFORESET_REENABLE; // 使能 FIFO 发送
    SciaRegs.SCIFFRX.bit.RXFIFORESET = SCIFFRX_RXFIFORESET_REENABLE; // 使能 FIFO 接收
    SciaRegs.SCIFFTX.bit.SCIRST = SCIFFTX_SCIRST_RESUME; // SCI 启用

    SciaRegs.SCIFFTX.bit.SCIFFENA = SCIFFTX_SCIFFENA_ENABLE; // 使能 SCI FIFO
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = SCIFFTX_TXFFINTCLR_CLEAR; // 清除发送中断标志
    SciaRegs.SCIFFTX.bit.TXFFIENA = SCIFFTX_TXFFIENA_DISABLE;   // 发送中断失能
    SciaRegs.SCIFFTX.bit.TXFFIL = 0;

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = SCIFFRX_RXFFOVRCLR_CLEAR; // 清除溢出标志
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = SCIFFRX_RXFFINTCLR_CLEAR; // 清除接收中断标志
    SciaRegs.SCIFFRX.bit.RXFFIENA = SCIFFRX_RXFFIENA_DISABLE; // 接收中断失能
    SciaRegs.SCIFFRX.bit.RXFFIL = 0; // 接收中断的级别

    SciaRegs.SCIFFCT.bit.ABDCLR = SCIFFCT_ABD_CLEAR; // 清除 ABD 完成标志位
}

void Init_SCIB_FIFO(void){


}
/* *******************************************************
 *
 * 初始化 SCIA
 *
 * *******************************************************/
/*
 * 初始化 SCIA 过程如下
 * 1、1 stop bit / No parity / No loopback mode / Idle mode / 8 char bits
 * 2、重置 (SCICTL2和SCIRXST) SCI
 * 3、禁用接收错误中断 / 选择 TX 唤醒方式 / 睡眠模式 / 使能 SCI 传输/接收
 * 3、禁止 发送/接收 SCI 中断
 * 4、设置波特率
 * 5、使能 SCI
 */
void Init_SCIA(void){

    SciaRegs.SCICCR.bit.STOPBITS = SCICCR_STOPBITS_ONE;           // 1位停止(VOFA+中通信可选)
    SciaRegs.SCICCR.bit.PARITYENA = SCICCR_PARITYENA_DISABLED;    // No parity
    SciaRegs.SCICCR.bit.LOOPBKENA = SCICCR_LOOPBKENA_DISABLED;    // No loopback mode
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE = SCICCR_ADDRIDLE_IDLE;     // Idle mode
    SciaRegs.SCICCR.bit.SCICHAR = SCICCR_SCICHAR_LENGTH_8;        // 8 char bits

    SciaRegs.SCICTL1.bit.RXERRINTENA = SCICTL1_RXERRINTENA_DISABLED;// Receive error interrupt disabled
    SciaRegs.SCICTL1.bit.SWRESET = SCICTL1_SWRESET_RESET;           // Reset flag
    SciaRegs.SCICTL1.bit.TXWAKE = SCICTL1_TXWAKE_NOT_SELECTD;       // 选择 TX 唤醒方式
    SciaRegs.SCICTL1.bit.SLEEP = SCICTL1_SLEEP_DISABLED;            // Disable sleep 睡眠模式禁用
    SciaRegs.SCICTL1.bit.TXENA = SCICTL1_TXENA_ENABLED;             // Enable TX
    SciaRegs.SCICTL1.bit.RXENA = SCICTL1_RXENA_ENABLED;             // Enable RX


    //SciaRegs.SCICTL2.bit.TXINTENA = SCICTL2_TXINTENA_ENABLED;       // Disable TXRDY interrupt
    //SciaRegs.SCICTL2.bit.RXBKINTENA = SCICTL2_RXBKINTENA_ENABLED;   // Disable RXRDY/BRKDT interrupt

    // SCI和SPI模块可以以比CPU频率慢得多的比特率进行通信。
    // 这些模块连接到一个共享的时钟分频器，它会产生一个来自SYSCLK的低速外围时钟（LSPCLK）。
    // LSPCLK默认使用一个/4分频器 @LSPCLK = 25 MHz (100 MHz SYSCLK)，但是这个比率可以通过LOSPCP寄存器来改变。
    // 每个SCI和SPI模块的时钟(PERx.LSPCLK)可以通过PCLKCRx寄存器进行独立门控。
    CpuSysRegs.PCLKCR7.bit.SCI_A = Turn_ON;

    // 波特率 = 9600 详见P2197 2206
    // 内部生成的串行时钟由低速外围时钟LSPCLK)和波特率选择寄存器决定。
    // SCI使用波特率选择寄存器的16位值来为一个给定的LSPCLK选择64K个不同的串行时钟速率中的一个。
    // 最大波特率 = LSPCLK/16
    // 默认使用 @LSPCLK = 25 MHz (100 MHz SYSCLK)

    // LSPCLK / (SCI Asynchronous Baud * 8) - 1
    // 25 000 000 /(9600*8) - 1 = 324
    //
    //SciaRegs.SCIHBAUD.all = 0x0001;
    //SciaRegs.SCILBAUD.all = 0x0044;

    // 25 000 000 /(115200*8) - 1 = 26
//    SciaRegs.SCIHBAUD.all = 0x0000;
//    SciaRegs.SCILBAUD.all = 0x001A;

    // 25 000 000 /(115200*8) - 1 = 2
    SciaRegs.SCIHBAUD.all = 0x0000;
    SciaRegs.SCILBAUD.all = 0x0002;

    //SciaRegs.SCICCR.bit.LOOPBKENA = SCICCR_LOOPBKENA_ENABLED; // Enable loop_back mode
    SciaRegs.SCICTL1.bit.SWRESET = SCICTL1_SWRESET_REENABLED;//  re-enable the SCI
}

//
// transmitSCIAChar - Transmit a character from the SCI
//
void transmitSCIAChar(uint16_t a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
    {

    }
    SciaRegs.SCITXBUF.all = a;
}

//
// transmitSCIAMessage - Transmit message via SCIA
//
void transmitSCIAMessage(unsigned char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        transmitSCIAChar(msg[i]);
        i++;
    }
}



