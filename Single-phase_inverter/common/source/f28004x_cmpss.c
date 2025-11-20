/*
 *  ======= f28004x_cmpss ========
 *  f28004x_cmpss target-side implementation
 *
 *  Created on: 2022年5月28日
 *  Author:     jzy
 *
 *
 *  F280049C 共有 7 组 比较器，每组比较器分别拥有 2 个比较器
 */
//
// Included Files
//
#include "f28004x_device.h"
#include "f28004x_examples.h"

// 自己添加的函数
/* --------------------------------------------------------------
 * 用户根据自己的实际需求修改
 * --------------------------------------------------------------*/

/* *******************************************************
 * 初始化并配置 CMPSS1 的高比较器
 *
 * DAC 的值使用斜坡发生器
 *
 * ILFB_AVG  ADCINA2  (heavily filtered average)
 * 比较引脚不能悬空
 *
 * *******************************************************/
void Init_CMPSS1H(void){

    EALLOW;


    // 1、使能 DAC
    Cmpss1Regs.COMPCTL.bit.COMPDACE = DAC_ENABLE;

    // 2、 配置比较器 CMP1H 的正极

    //配置 ADCINA2 引脚为比较器 CMP1H 的正极
    //其他配置请参考 Technical Reference Manual
    //12.1.3 Analog Pins and Internal Connections
    AnalogSubsysRegs.CMPHPMXSEL.bit.CMP1HPMXSEL = 0;

    // 3、配置比较器 CMP1H 的负极

    // 配置 比较器 CMP1H 的负极 为内部 DAC
    Cmpss1Regs.COMPCTL.bit.COMPHSOURCE = NEGSOURCE_DAC;

    // 4、配置比较器 CMP1H 的输出是否反极

    // 配置 比较器 CMP1H 的输出不反极
    Cmpss1Regs.COMPCTL.bit.COMPHINV = INV_DISABLE;

    // 5、配置比较器 CMP1H 的 ASYNCHEN

    // 配置 比较器 CMP1H 的 ASYNCHEN
    Cmpss1Regs.COMPCTL.bit.ASYNCHEN = ASYNC_DISABLE;

    // 6、配置比较器 CMP1H 的负极参考源、参考值及更新时钟

    // 配置 DAC 的值
    //Cmpss1Regs.DACHVALS.bit.DACVAL = 1024;//248

    // ----------------------------------------------------------------
    // 配置斜坡发生器
    // ----------------------------------------------------------------
    // 配置斜坡发生器 同步时钟
    Cmpss1Regs.COMPDACCTL.bit.RAMPSOURCE = RAMPSOURCE_EPWM1SYNCPER;
    // 斜坡发生器装载使用 Shadow Register
    Cmpss1Regs.COMPDACCTL.bit.RAMPLOADSEL = RAMPLOAD_RAMPMAXREFS;
    // 设置斜坡发生器 Shadow Register 的最大值 0-15 bit
    Cmpss1Regs.RAMPMAXREFS = 10;
    // 设置斜坡发生器 减数 Shadow Register 的减数值 0-15 bit
    Cmpss1Regs.RAMPDECVALS = 1;
    // 设置斜坡发生器 Shadow Register 的延时值 0-12 bit
    Cmpss1Regs.RAMPDLYS.bit.DELAY = 1;
    // ----------------------------------------------------------------

    // 配置 DAC 的源为斜坡发生器
    Cmpss1Regs.COMPDACCTL.bit.DACSOURCE = DACSOURCE_RAMP;
    // 配置 DAC 的参考源 VDDA
    Cmpss1Regs.COMPDACCTL.bit.SELREF = DAC_REF_VDDA;
    // 配置 DAC 的值更新时钟为 SYSCLK
    Cmpss1Regs.COMPDACCTL.bit.SWLOADSEL = SWLOAD_SYSCLK;


//    // 7、配置数字滤波器--------------------------------------------------
//    // 如果配置不好，会造成比较器太灵敏
//
//    // 配置滤波器阈值 0-31
//    Cmpss1Regs.CTRIPHFILCTL.bit.THRESH = 29;
//    // 配置滤波器样本窗口大小 0-31
//    Cmpss1Regs.CTRIPHFILCTL.bit.SAMPWIN = 30;
//    // 配置滤波器分频系数 0-1023
//    Cmpss1Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = 1000;
//    // 初始化滤波器输入样本
//    Cmpss1Regs.CTRIPHFILCTL.bit.FILINIT = FILINIT_INIT ;
    // --------------------------------------------------------------------

    // 8、清零 滤波器 输出锁存器
    Cmpss1Regs.COMPSTSCLR.bit.HLATCHCLR = HLATCHCLR_RESET;

    // 9、使能 EPWMSYNCPER 清零 输出锁存器
    Cmpss1Regs.COMPSTSCLR.bit.HSYNCCLREN = HSYNCCLREN_ENABLE;

    // 10、配置 CTRIPH to ePWM X-BAR 的源为数字滤波器输出
    Cmpss1Regs.COMPCTL.bit.CTRIPHSEL = CTRIP_FILTER; // (ePWM X-BAR only)
    // 配置 CTRIPOUTL to OUTPUT X-BAR 的源为数字滤波器输出
    //Cmpss1Regs.COMPCTL.bit.CTRIPOUTHSEL = CTRIP_FILTER;


    //
    // Use the Blanking signal from Sync Buck EPWM to reject switching noise
    //
    Cmpss1Regs.COMPDACCTL.bit.BLANKSOURCE = BLANKSOURCE_EPWM1BLANK;
    Cmpss1Regs.COMPDACCTL.bit.BLANKEN = EPWMBLANK_ENABLE;

    //
    // Comparator hysteresis control
    //
    Cmpss1Regs.COMPHYSCTL.bit.COMPHYS = HYS_2TYPICAL;

    EDIS;
}
///* *******************************************************
// * 初始化并配置 CMPSS1 的高比较器
// *
// * DAC 的值不使用斜坡发生器
// *
// * ILFB_AVG  ADCINA2  (heavily filtered average)
// * 比较引脚不能悬空
// *
// * *******************************************************/
//void Init_CMPSS1H(void){
//
//    EALLOW;
//
//
//    // 1、使能 DAC
//    Cmpss1Regs.COMPCTL.bit.COMPDACE = DAC_ENABLE;
//
//    // 2、 配置比较器 CMP1H 的正极
//
//    //配置 ADCINA2 引脚为比较器 CMP1H 的正极
//    //其他配置请参考 Technical Reference Manual
//    //12.1.3 Analog Pins and Internal Connections
//    AnalogSubsysRegs.CMPHPMXSEL.bit.CMP1HPMXSEL = 0;
//
//    // 3、配置比较器 CMP1H 的负极
//
//    // 配置 比较器 CMP1H 的负极 为内部 DAC
//    Cmpss1Regs.COMPCTL.bit.COMPHSOURCE = NEGSOURCE_DAC;
//
//    // 4、配置比较器 CMP1H 的输出是否反极
//
//    // 配置 比较器 CMP1H 的输出不反极
//    Cmpss1Regs.COMPCTL.bit.COMPHINV = INV_DISABLE;
//
//    // 5、配置比较器 CMP1H 的 ASYNCHEN
//
//    // 配置 比较器 CMP1H 的 ASYNCHEN
//    Cmpss1Regs.COMPCTL.bit.ASYNCHEN = ASYNC_DISABLE;
//
//    // 6、配置比较器 CMP1H 的负极参考源、参考值及更新时钟
//
//    // 配置 DAC 的值
//    Cmpss1Regs.DACHVALS.bit.DACVAL = 1024;//248
//    // 配置 DAC 的源
//    Cmpss1Regs.COMPDACCTL.bit.DACSOURCE = DACSOURCE_DACHVALS;
//    // 配置 DAC 的参考源 VDDA
//    Cmpss1Regs.COMPDACCTL.bit.SELREF = DAC_REF_VDDA;
//    // 配置 DAC 的值更新时钟为 SYSCLK
//    Cmpss1Regs.COMPDACCTL.bit.SWLOADSEL = SWLOAD_SYSCLK;
//
//    // 7、配置比较器输出数字滤波器
//    // 如果配置不好，会造成比较器太灵敏
//
//    // 配置滤波器阈值 0-31
//    Cmpss1Regs.CTRIPHFILCTL.bit.THRESH = 29;
//    // 配置滤波器样本窗口大小 0-31
//    Cmpss1Regs.CTRIPHFILCTL.bit.SAMPWIN = 30;
//    // 配置滤波器分频系数 0-1023
//    Cmpss1Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = 1000;
//    // 初始化滤波器输入样本
//    Cmpss1Regs.CTRIPHFILCTL.bit.FILINIT = FILINIT_INIT ;
//    // 初始化滤波器输出锁存器
//    Cmpss1Regs.COMPSTSCLR.bit.HLATCHCLR = HLATCHCLR_RESET;
//
//    // 配置 CTRIPH to ePWM X-BAR 的源为数字滤波器输出
//    Cmpss1Regs.COMPCTL.bit.CTRIPHSEL = CTRIP_FILTER; // (ePWM X-BAR only)
//    // 配置 CTRIPOUTL to OUTPUT X-BAR 的源为数字滤波器输出
//    //Cmpss1Regs.COMPCTL.bit.CTRIPOUTHSEL = CTRIP_FILTER;
//
//
//    //
//    // Use the Blanking signal from Sync Buck EPWM to reject switching noise
//    //
//    Cmpss1Regs.COMPDACCTL.bit.BLANKSOURCE = BLANKSOURCE_EPWM1BLANK;
//    Cmpss1Regs.COMPDACCTL.bit.BLANKEN = EPWMBLANK_ENABLE;
//
//    //
//    // Comparator hysteresis control
//    //
//    Cmpss1Regs.COMPHYSCTL.bit.COMPHYS = HYS_2TYPICAL;
//
//    EDIS;
//}

/* *******************************************************
 * 初始化并配置 CMPSS1 的低比较器
 * ILFB      ADCINA3  (over current protection)
 * 比较引脚不能悬空
 *
 * *******************************************************/
void Init_CMPSS1L(void){

    EALLOW;

    // 1、使能 DAC
    Cmpss1Regs.COMPCTL.bit.COMPDACE = DAC_ENABLE;

    // 2、 配置比较器 CMP1L 的正极

    //配置 ADCINA3 引脚为比较器 CMP1L 的正极
    //其他配置请参考 Technical Reference Manual
    //12.1.3 Analog Pins and Internal Connections
    AnalogSubsysRegs.CMPLPMXSEL.bit.CMP1LPMXSEL = 3;

    // 3、配置比较器 CMP1L 的负极

    Cmpss1Regs.COMPCTL.bit.COMPLSOURCE = NEGSOURCE_DAC;// 配置 比较器 CMP1L 的负极 为内部 DAC

    // 4、配置比较器 CMP1L 的输出是否反极

    Cmpss1Regs.COMPCTL.bit.COMPLINV = INV_DISABLE;// 配置 比较器 CMP1L 的输出不反极

    // 5、配置比较器 CMP1L 的 ASYNCLEN

    Cmpss1Regs.COMPCTL.bit.ASYNCLEN = ASYNC_DISABLE;// 配置 比较器 CMP1L 的 ASYNCLEN 无效

    // 6、配置比较器 CMP1L 的负极参考源、参考值及更新时钟

    Cmpss1Regs.DACLVALS.bit.DACVAL = 1024;// 配置 DAC 的值 最大 4096

    //Cmpss1Regs.COMPDACCTL.bit.DACSOURCE = DACSOURCE_DACHVALS;// CMP1L 没有 DAC 的源选择 不用配

    Cmpss1Regs.COMPDACCTL.bit.SELREF = DAC_REF_VDDA;// 配置 DAC 的参考源 VDDA

    Cmpss1Regs.COMPDACCTL.bit.SWLOADSEL = SWLOAD_SYSCLK;// 配置 DAC 的值更新时钟为 SYSCLK

    // 7、配置比较器输出数字滤波器
    // 如果配置不好，会造成比较器太灵敏

    // 配置滤波器阈值 0-31
    Cmpss1Regs.CTRIPLFILCTL.bit.THRESH = 29;
    // 配置滤波器样本窗口大小 0-31
    Cmpss1Regs.CTRIPLFILCTL.bit.SAMPWIN = 30;
    // 配置过滤器分频系数 0-1023
    Cmpss1Regs.CTRIPLFILCLKCTL.bit.CLKPRESCALE = 1000;
    // 初始化滤波器输入样本
    Cmpss1Regs.CTRIPLFILCTL.bit.FILINIT = FILINIT_INIT ;
    // 初始化滤波器输出锁存器
    Cmpss1Regs.COMPSTSCLR.bit.LLATCHCLR = LLATCHCLR_RESET;

    // 配置 CTRIPH to ePWM X-BAR 的源为数字滤波器输出
    Cmpss1Regs.COMPCTL.bit.CTRIPLSEL = CTRIP_FILTER; // (ePWM X-BAR only)
    // 配置 CTRIPOUTL to OUTPUT X-BAR 的源为数字滤波器输出
    //Cmpss1Regs.COMPCTL.bit.CTRIPOUTLSEL = CTRIP_FILTER;

    //
    // Use the Blanking signal from Sync Buck EPWM to reject switching noise
    //
    Cmpss1Regs.COMPDACCTL.bit.BLANKSOURCE = BLANKSOURCE_EPWM1BLANK;
    Cmpss1Regs.COMPDACCTL.bit.BLANKEN = EPWMBLANK_ENABLE;

    //
    // Comparator hysteresis control
    //
    Cmpss1Regs.COMPHYSCTL.bit.COMPHYS = HYS_2TYPICAL;

    EDIS;
}
