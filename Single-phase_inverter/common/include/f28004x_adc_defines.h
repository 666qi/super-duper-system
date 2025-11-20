//###########################################################################
//
// FILE:   f28004x_adc_defines.h
//
// TITLE:  #defines used in ADC examples
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

#ifndef F28004X_ADC_DEFINES_H
#define F28004X_ADC_DEFINES_H

#ifdef __cplusplus
extern "C" {
#endif

//
// Defines
//

//
// Definitions for specifying an ADC
//
#define ADC_ADCA        0
#define ADC_ADCB        1
#define ADC_ADCC        2
#define ADC_ADCD        3

//
// Definitions for specifying reference mode
//
#define ADC_INTERNAL    0
#define ADC_EXTERNAL    1

//
// Definitions for specifying reference voltage
//
#define ADC_VREF3P3     0
#define ADC_VREF2P5     1

// ---------------------------------
#define ADC_INT_REF_TSSLOPE  (*(int16_t *)((uintptr_t)0x705BD))
#define ADC_INT_REF_TSOFFSET (*(int16_t *)((uintptr_t)0x705BE))
#define ADC_EXT_REF_TSSLOPE  (*(int16_t *)((uintptr_t)0x705BF))
#define ADC_EXT_REF_TSOFFSET (*(int16_t *)((uintptr_t)0x705C0))


// ADCCTL1 Register (Offset = 0h) [reset = 0h]
//////////////////////////////////////////////////////////////////////////
// ADCPWDNZ bit
// ADC Power Down (active low)
// This bit controls the power up and
// power down of all the analog circuitry inside the analog core
// --------------------------------------------------------
#define Power_Down    0
#define Power_Up      1


// INTPULSEPOS bit
// ADC Interrupt Pulse Position
// --------------------------------------------------------
#define ADC_COMPLETE_BEGIN    0
#define ADC_COMPLETE_END      1

// ADCCTL2 Register (Offset = 1h) [reset = 0h]
//////////////////////////////////////////////////////////////////////////
// ADC Clock Prescaler
/*
0000 ADCCLK = Input Clock / 1.0
0001 Reserved
0010 ADCCLK = Input Clock / 2.0
0011 Reserved
0100 ADCCLK = Input Clock / 3.0
0101 Reserved
0110 ADCCLK = Input Clock / 4.0
0111 Reserved
1000 ADCCLK = Input Clock / 5.0
1001 Reserved
1010 ADCCLK = Input Clock / 6.0
1011 Reserved
1100 ADCCLK = Input Clock / 7.0
1101 Reserved
1110 ADCCLK = Input Clock / 8.0
1111 Reserved
*/
// --------------------------------------------------------
#define ADC_Clock_Prescaler_1    0
#define ADC_Clock_Prescaler_2    2
#define ADC_Clock_Prescaler_3    4
#define ADC_Clock_Prescaler_4    6
#define ADC_Clock_Prescaler_5    8
#define ADC_Clock_Prescaler_6    10
#define ADC_Clock_Prescaler_7    12
#define ADC_Clock_Prescaler_8    14

// ADCBURSTCTL Register (Offset = 2h) [reset = 0h]
//////////////////////////////////////////////////////////////////////////
// SOC Burst Mode Enable
// --------------------------------------------------------
#define ADC_BURSTEN_DISABLE      0
#define ADC_BURSTEN_ENABLE       1

// SOC Burst Size Select
// --------------------------------------------------------
#define ADC_BURSTSIZE1      0
#define ADC_BURSTSIZE2      1
#define ADC_BURSTSIZE3      2
#define ADC_BURSTSIZE4      3
#define ADC_BURSTSIZE5      4
#define ADC_BURSTSIZE6      5
#define ADC_BURSTSIZE7      6
#define ADC_BURSTSIZE8      7
#define ADC_BURSTSIZE9      8
#define ADC_BURSTSIZE10     9
#define ADC_BURSTSIZE11     10
#define ADC_BURSTSIZE12     11
#define ADC_BURSTSIZE13     12
#define ADC_BURSTSIZE14     13
#define ADC_BURSTSIZE15     14
#define ADC_BURSTSIZE16     15

// SOC Burst Trigger Source Select
// --------------------------------------------------------
#define ADC_BURSTTRIGSEL_Software      0
#define ADC_BURSTTRIGSEL_Timer0        1
#define ADC_BURSTTRIGSEL_Timer1        2
#define ADC_BURSTTRIGSEL_Timer2        3
#define ADC_BURSTTRIGSEL_GPIO          4
#define ADC_BURSTTRIGSEL_ePWM1ADCSOCA  5
#define ADC_BURSTTRIGSEL_ePWM1ADCSOCB  6
#define ADC_BURSTTRIGSEL_ePWM2ADCSOCA  7
#define ADC_BURSTTRIGSEL_ePWM2ADCSOCB  8
#define ADC_BURSTTRIGSEL_ePWM3ADCSOCA  9
#define ADC_BURSTTRIGSEL_ePWM3ADCSOCB  10
#define ADC_BURSTTRIGSEL_ePWM4ADCSOCA  11
#define ADC_BURSTTRIGSEL_ePWM4ADCSOCB  12
#define ADC_BURSTTRIGSEL_ePWM5ADCSOCA  13
#define ADC_BURSTTRIGSEL_ePWM5ADCSOCB  14
#define ADC_BURSTTRIGSEL_ePWM6ADCSOCA  15
#define ADC_BURSTTRIGSEL_ePWM6ADCSOCB  16
#define ADC_BURSTTRIGSEL_ePWM7ADCSOCA  17
#define ADC_BURSTTRIGSEL_ePWM7ADCSOCB  18
#define ADC_BURSTTRIGSEL_ePWM8ADCSOCA  19
#define ADC_BURSTTRIGSEL_ePWM8ADCSOCB  20

// ADCINTFLGCLR Register (Offset = 4h) [reset = 0h]
//////////////////////////////////////////////////////////////////////////
// 中断标志清除位
#define ADC_ADCINTxFLG_CLEAR      1

// ADCINTOVFCLR Register (Offset = 6h) [reset = 0h]
//////////////////////////////////////////////////////////////////////////
// 中断溢出标志清除位
#define ADC_ADCINTxOVF_CLEAR      1

//
// ADCINTSEL1N2 Register (Offset = 7h) [reset = 0h]
// ADCINTSEL3N4 Register (Offset = 8h) [reset = 0h]
//////////////////////////////////////////////////////////////////////////
// --------------------------------------------------------
// INT1CONT INT2CONT INT3CONT INT4CONT bit
// ADCINTx Continue to Interrupt Mode
// --------------------------------------------------------
#define ADC_INTxCONT_DISABLE     0  // 连续中断禁止
#define ADC_INTxCONT_ENABLE      1  // 连续中断使能

// INT1E INT2E INT3E INT4E bit
// ADCINTx Interrupt Enable
// --------------------------------------------------------
#define ADC_INTx_DISABLE     0
#define ADC_INTx_ENABLE      1

// INT1SEL INT2SEL INT3SEL INT4SEL bit
// --------------------------------------------------------
#define ADC_INTxSEL_EOC0      0
#define ADC_INTxSEL_EOC1      1
#define ADC_INTxSEL_EOC2      2
#define ADC_INTxSEL_EOC3      3
#define ADC_INTxSEL_EOC4      4
#define ADC_INTxSEL_EOC5      5
#define ADC_INTxSEL_EOC6      6
#define ADC_INTxSEL_EOC7      7
#define ADC_INTxSEL_EOC8      8
#define ADC_INTxSEL_EOC9      9
#define ADC_INTxSEL_EOC10      10
#define ADC_INTxSEL_EOC11      11
#define ADC_INTxSEL_EOC12      12
#define ADC_INTxSEL_EOC13      13
#define ADC_INTxSEL_EOC14      14
#define ADC_INTxSEL_EOC15      15

// ADCSOCPRICTL Register (Offset = 9h) [reset = 200h]
//////////////////////////////////////////////////////////////////////////
// 9-5 bit RRPOINTER
// Round Robin Pointer
// --------------------------------------------------------
#define ADCSOC_RRPOINTER_SOC0     0  // SOC1 最高优先级
#define ADCSOC_RRPOINTER_SOC1     1  // SOC2 最高优先级
#define ADCSOC_RRPOINTER_SOC2     2  // SOC3 最高优先级
#define ADCSOC_RRPOINTER_SOC3     3  // SOC4 最高优先级
#define ADCSOC_RRPOINTER_SOC4     4  // SOC5 最高优先级
#define ADCSOC_RRPOINTER_SOC5     5  // SOC6 最高优先级
#define ADCSOC_RRPOINTER_SOC6     6  // SOC7 最高优先级
#define ADCSOC_RRPOINTER_SOC7     7  // SOC8 最高优先级
#define ADCSOC_RRPOINTER_SOC8     8  // SOC9 最高优先级
#define ADCSOC_RRPOINTER_SOC9     9  // SOC10 最高优先级
#define ADCSOC_RRPOINTER_SOC10    10 // SOC11 最高优先级
#define ADCSOC_RRPOINTER_SOC11    11 // SOC12 最高优先级
#define ADCSOC_RRPOINTER_SOC12    12 // SOC13 最高优先级
#define ADCSOC_RRPOINTER_SOC13    13 // SOC14 最高优先级
#define ADCSOC_RRPOINTER_SOC14    14 // SOC15 最高优先级
#define ADCSOC_RRPOINTER_SOC15    15 // SOC0  最高优先级
#define ADCSOC_RRPOINTER_Reset    16 // 重置 SOC0  最高优先级

// 4-0 bit SOCPRIORITY
// --------------------------------------------------------
#define ADCSOC_PRIORITY_RR           0    // SOC priority is handled in round robin mode for all channels
#define ADCSOC_PRIORITY_SOC0         1    // SOC0 is high priority, rest of channels are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC1    2    // SOC0-SOC1 are high priority, SOC2-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC2    3    // SOC0-SOC2 are high priority, SOC3-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC3    4    // SOC0-SOC3 are high priority, SOC4-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC4    5    // SOC0-SOC4 are high priority, SOC5-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC5    6    // SOC0-SOC5 are high priority, SOC6-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC6    7    // SOC0-SOC6 are high priority, SOC7-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC7    8    // SOC0-SOC7 are high priority, SOC8-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC8    9    // SOC0-SOC8 are high priority, SOC9-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC9    10   // SOC0-SOC9 are high priority, SOC10-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC10   11   // SOC0-SOC10 are high priority, SOC11-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC11   12   // SOC0-SOC11 are high priority, SOC12-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC12   13   // SOC0-SOC12 are high priority, SOC13-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC13   14   // SOC0-SOC13 are high priority, SOC14-SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC14   15   // SOC0-SOC14 are high priority, SOC15 are in round robin mode
#define ADCSOC_PRIORITY_SOC0_SOC15   16   // All SOCs are in high priority mode, arbitrated by SOC number. Others Invalid selection

// ADCINTSOCSEL1 Register (Offset = Ah) [reset = 0h]
// ADCINTSOCSEL2 Register (Offset = Bh) [reset = 0h]
//////////////////////////////////////////////////////////////////////////
#define ADCINT_Trigger_DISABLE     0
#define ADCINT1_Trigger_SOC        1
#define ADCINT2_Trigger_SOC        2

// ADCSOCFRC1 Register (Offset = Dh) [reset = 0h]
// 强制转换启动寄存器
//////////////////////////////////////////////////////////////////////////
#define ADC_ADCSOCFRC1_stop      0
#define ADC_ADCSOCFRC1_start     1

// ADCSOCOVFCLR1 Register (Offset = Fh) [reset = 0h]
//////////////////////////////////////////////////////////////////////////
#define ADC_ADCSOCxOVF_CLEAR      1

// ADCSOC0CTL Register (Offset = 10h) [reset = 0h]
// ADCSOC1CTL Register (Offset = 12h) [reset = 0h]
// ADCSOC2CTL Register (Offset = 14h) [reset = 0h]
// ADCSOC3CTL Register (Offset = 16h) [reset = 0h]
// ADCSOC4CTL Register (Offset = 18h) [reset = 0h]
// ADCSOC5CTL Register (Offset = 1Ah) [reset = 0h]
// ADCSOC6CTL Register (Offset = 1Ch) [reset = 0h]
// ADCSOC7CTL Register (Offset = 1Eh) [reset = 0h]
// ADCSOC8CTL Register (Offset = 20h) [reset = 0h]
// ADCSOC9CTL Register (Offset = 22h) [reset = 0h]
// ADCSOC10CTL Register (Offset = 24h) [reset = 0h]
// ADCSOC11CTL Register (Offset = 26h) [reset = 0h]
// ADCSOC12CTL Register (Offset = 28h) [reset = 0h]
// ADCSOC13CTL Register (Offset = 2Ah) [reset = 0h]
// ADCSOC14CTL Register (Offset = 2Ch) [reset = 0h]
// ADCSOC15CTL Register (Offset = 2Eh) [reset = 0h]
//////////////////////////////////////////////////////////////////////////
// CHSEL bit
// --------------------------------------------------------
#define ADC_CHSEL_ADCIN0      0
#define ADC_CHSEL_ADCIN1      1
#define ADC_CHSEL_ADCIN2      2
#define ADC_CHSEL_ADCIN3      3
#define ADC_CHSEL_ADCIN4      4
#define ADC_CHSEL_ADCIN5      5
#define ADC_CHSEL_ADCIN6      6
#define ADC_CHSEL_ADCIN7      7
#define ADC_CHSEL_ADCIN8      8
#define ADC_CHSEL_ADCIN9      9
#define ADC_CHSEL_ADCIN10      10
#define ADC_CHSEL_ADCIN11      11
#define ADC_CHSEL_ADCIN12      12
#define ADC_CHSEL_ADCIN13      13
#define ADC_CHSEL_ADCIN14      14
#define ADC_CHSEL_ADCIN15      15

// TRIGSEL bit
/*
 * 00h ADCTRIG0 - Software only
 * 01h ADCTRIG1 - CPU1 Timer 0, TINT0n
 * 02h ADCTRIG2 - CPU1 Timer 1, TINT1n
 * 03h ADCTRIG3 - CPU1 Timer 2, TINT2n
 * 04h ADCTRIG4 - GPIO, ADCEXTSOC
 * 05h ADCTRIG5 - ePWM1, ADCSOCA
 * 06h ADCTRIG6 - ePWM1, ADCSOCB
 * 07h ADCTRIG7 - ePWM2, ADCSOCA
 * 08h ADCTRIG8 - ePWM2, ADCSOCB
 * 09h ADCTRIG9 - ePWM3, ADCSOCA
 * 0Ah ADCTRIG10 - ePWM3, ADCSOCB
 * 0Bh ADCTRIG11 - ePWM4, ADCSOCA
 * 0Ch ADCTRIG12 - ePWM4, ADCSOCB
 * 0Dh ADCTRIG13 - ePWM5, ADCSOCA
 * 0Eh ADCTRIG14 - ePWM5, ADCSOCB
 * 0Fh ADCTRIG15 - ePWM6, ADCSOCA
 * 10h ADCTRIG16 - ePWM6, ADCSOCB
 * 11h ADCTRIG17 - ePWM7, ADCSOCA
 * 12h ADCTRIG18 - ePWM7, ADCSOCB
 * 13h ADCTRIG19 - ePWM8, ADCSOCA
 * 14h ADCTRIG20 - ePWM8, ADCSOCB
 */
// --------------------------------------------------------
#define ADC_TRIGSEL_Software      0
#define ADC_TRIGSEL_Timer0        1
#define ADC_TRIGSEL_Timer1        2
#define ADC_TRIGSEL_Timer2        3
#define ADC_TRIGSEL_GPIO          4
#define ADC_TRIGSEL_ePWM1ADCSOCA  5
#define ADC_TRIGSEL_ePWM1ADCSOCB  6
#define ADC_TRIGSEL_ePWM2ADCSOCA  7
#define ADC_TRIGSEL_ePWM2ADCSOCB  8
#define ADC_TRIGSEL_ePWM3ADCSOCA  9
#define ADC_TRIGSEL_ePWM3ADCSOCB  10
#define ADC_TRIGSEL_ePWM4ADCSOCA  11
#define ADC_TRIGSEL_ePWM4ADCSOCB  12
#define ADC_TRIGSEL_ePWM5ADCSOCA  13
#define ADC_TRIGSEL_ePWM5ADCSOCB  14
#define ADC_TRIGSEL_ePWM6ADCSOCA  15
#define ADC_TRIGSEL_ePWM6ADCSOCB  16
#define ADC_TRIGSEL_ePWM7ADCSOCA  17
#define ADC_TRIGSEL_ePWM7ADCSOCB  18
#define ADC_TRIGSEL_ePWM8ADCSOCA  19
#define ADC_TRIGSEL_ePWM8ADCSOCB  20


// ACQPS bit
// SOC0 Acquisition Prescale. Controls the sample and hold window for this SOC.
/*
000h Sample window is 1 system clock cycle wide
001h Sample window is 2 system clock cycles wide
002h Sample window is 3 system clock cycles wide
...
1FFh Sample window is 512 system clock cycles wide
 */

// ADCEVTCLR Register (Offset = 32h) [reset = 0h]
// ADC Event Clear Register
//////////////////////////////////////////////////////////////////////////
// PPBxZERO
// --------------------------------------------------------
#define PPBxZERO_EventClear       1 // 过零清除

// PPBxTRIPLO
// --------------------------------------------------------
#define PPBxTRIPLO_EventClear     1 // 跳闸低清除

// PPBxTRIPHI
// --------------------------------------------------------
#define PPBxTRIPHI_EventClear     1 // 跳闸高清除

// ADCEVTSEL Register (Offset = 34h) [reset = 0h]
// ADC 事件选择
//////////////////////////////////////////////////////////////////////////
// PPBxZERO
// --------------------------------------------------------
#define PPBxZERO_EventDISABLE      0
#define PPBxZERO_EventENABLE       1

// PPBxTRIPLO
// --------------------------------------------------------
#define PPBxTRIPLO_EventDISABLE      0
#define PPBxTRIPLO_EventENABLE       1

// PPBxTRIPHI
// --------------------------------------------------------
#define PPBxTRIPHI_EventDISABLE      0
#define PPBxTRIPHI_EventENABLE       1

// ADCEVTINTSEL Register (Offset = 36h) [reset = 0h]
//ADC 事件中断选择寄存器
//////////////////////////////////////////////////////////////////////////
// PPBxZERO
// --------------------------------------------------------
#define PPBxZERO_INTDISABLE      0
#define PPBxZERO_INTENABLE       1

// PPBxTRIPLO
// --------------------------------------------------------
#define PPBxTRIPLO_INTDISABLE      0
#define PPBxTRIPLO_INTENABLE       1

// PPBxTRIPHI
// --------------------------------------------------------
#define PPBxTRIPHI_INTDISABLE      0
#define PPBxTRIPHI_INTENABLE       1

// ADCOSDETECT Register (Offset = 38h) [reset = 0h]
// ADC 开路和短路检测寄存器
//////////////////////////////////////////////////////////////////////////
// DETECTCFG 2-0
// --------------------------------------------------------
/*
ADC Opens and Shorts Detect Configuration. This bit field defines
the open/shorts detection circuit state.
0h Open/Shorts detection circuit is disabled.
1h Open/Shorts detection circuit is enabled at zero scale.
2h Open/Shorts detection circuit is enabled at full scale.
3h Open/Shorts detection circuit is enabled at (nominal) 5/12 scale.
4h Open/Shorts detection circuit is enabled at (nominal) 7/12 scale.
5h Open/Shorts detection circuit is enabled with a (nominal) 5K
pulldown to VSSA.
6h Open/Shorts detection circuit is enabled with a (nominal) 5K
pullup to VDDA.
7h Open/Shorts detection circuit is enabled with a (nominal) 7K
pulldown to VSSA.
 */

// ADCOFFTRIM Register (Offset = 3Bh) [reset = 0h]
// ADC偏移调整寄存器
// 以 16 为基数可以向上或向下调整转换器的转换结果，以补偿 ADC 的误差。
//////////////////////////////////////////////////////////////////////////

// OFFTRIM 7-0
// --------------------------------------------------------
/*
7 LSB (16*7) = 112
6 LSB (16*6) = 96
5 LSB (16*5) = 80
4 LSB (16*4) = 64
3 LSB (16*3) = 48
2 LSB (16*2) = 32
1 LSB (16*1) = 16
0 LSB (16*0) = 0
-1 LSB (16*(-1)) = 240
:
:
-7LSB(16*(-7)) = 144
 */

// ADCPPB1CONFIG Register (Offset = 40h) [reset = 0h]
// ADCPPB2CONFIG Register (Offset = 48h) [reset = 0h]
// ADCPPB3CONFIG Register (Offset = 50h) [reset = 0h]
// ADCPPB4CONFIG Register (Offset = 58h) [reset = 0h]
// ADC 后处理模块
//////////////////////////////////////////////////////////////////////////
// 5 CBCEN
// ADC Post Processing Block x Cycle By Cycle Enable
// --------------------------------------------------------
#define PPBxCONFIG_CBCEN_DISABLE      0
#define PPBxCONFIG_CBCEN_ENABLE       1

// 4 TWOSCOMPEN
// ADC Post Processing Block x Two's Complement Enable.
// --------------------------------------------------------
#define PPBxCONFIG_TWOSCOMPEN_POS      0  // 0 ADCPPB1RESULT = ADCRESULTx - ADCPPB1OFFREF
#define PPBxCONFIG_TWOSCOMPEN_NEG      1  // 1 ADCPPB1RESULT = ADCPPB1OFFREF - ADCRESULTx

// 3-0 CONFIG
// ADC Post Processing Block x Configuration
// --------------------------------------------------------
#define PPBxCONFIG_CONFIG_SOC0      0
#define PPBxCONFIG_CONFIG_SOC1      1
#define PPBxCONFIG_CONFIG_SOC2      2
#define PPBxCONFIG_CONFIG_SOC3      3
#define PPBxCONFIG_CONFIG_SOC4      4
#define PPBxCONFIG_CONFIG_SOC5      5
#define PPBxCONFIG_CONFIG_SOC6      6
#define PPBxCONFIG_CONFIG_SOC7      7
#define PPBxCONFIG_CONFIG_SOC8      8
#define PPBxCONFIG_CONFIG_SOC9      9
#define PPBxCONFIG_CONFIG_SOC10     10
#define PPBxCONFIG_CONFIG_SOC11     11
#define PPBxCONFIG_CONFIG_SOC12     12
#define PPBxCONFIG_CONFIG_SOC13     13
#define PPBxCONFIG_CONFIG_SOC14     14
#define PPBxCONFIG_CONFIG_SOC15     15




// ANALOG_SUBSYS_REGS Registers
/*
 * AnalogSubsysRegs
 * ANALOG_SUBSYS_REGS
 * Start Address 0x0005_D700
 * End Address   0x0005_D7FF
 */
//////////////////////////////////////////////////////////////////////////

// ANAREFPP Register (Offset = 1Eh) [reset = 0h]
// ADC Analog Reference Peripheral Properties register.

//////////////////////////////////////////////////////////////////////////
// ANAREFCDIS 1 bit
// ANAREFC Disable.  禁用 ANAREFC
// Note: This bit should be programmed to 1 in parts where VREFHIA
// and VREFHIB are double bonded.
// --------------------------------------------------------
#define ANAREFCDIS_ENABLE  0
#define ANAREFCDIS_DISABLE 1

// ANAREFBDIS 0 bit
// ANAREFB Disable.  禁用ANAREFB
// Note: This bit should be programmed to 1 in parts where VREFHIA,
// VREFHIB and VREFHIC are triple bonded.
#define ANAREBCDIS_ENABLE  0
#define ANAREFBDIS_DISABLE 1


// TSNSCTL Register (Offset = 60h) [reset = 0h]
// Temperature Sensor Control Register
// Temperature Sensor Enable. This bit enables the temperature sensor output to the ADC.
//////////////////////////////////////////////////////////////////////////
// ENABLE 0 bit
// --------------------------------------------------------
#define TSNSCTL_DISABLE      0
#define TSNSCTL_ENABLE       1

// ANAREFCTL Register (Offset = 68h) [reset = Fh]
// Analog Reference Control Register
//////////////////////////////////////////////////////////////////////////
// ANAREFC2P5SEL 10 bit
// ANAREFB2P5SEL  9 bit
// ANAREFA2P5SEL  8 bit
// Analog referenc B 2.5V source select. 2.5V源选择
// --------------------------------------------------------
#define ANAREFx2P5SEL_1V6    0  // 0 Internal 1.65V reference mode (3.3V reference range)
#define ANAREFx2P5SEL_2V5    1  // 1 Internal 2.5V reference mode (2.5V reference range)

// ANAREFCSEL 2 bit
// ANAREFBSEL 1 bit
// ANAREFASEL 0 bit
// Analog reference x mode select. 模式态选择
// --------------------------------------------------------
#define ANAREFxSEL_Internal   0
#define ANAREFxSEL_Externa    1

// VMONCTL Register (Offset = 70h) [reset = 0h]
// Voltage Monitor Control Register 电压监视器控制寄存器
//////////////////////////////////////////////////////////////////////////
// BORLVMONDIS 8 bit
// BORL disable on VDDIO
// --------------------------------------------------------
#define BORLVMONDIS_ENABLE   0    // 0 BORL is enabled on VDDIO
#define BORLVMONDIS_DISABLE  1    // 1 BORL is disabled on VDDIO

// DCDCCTL Register (Offset = 78h) [reset = 80000000h]
// DC-DC control register.
//////////////////////////////////////////////////////////////////////////
// DCDCEN  0 bit
// Enable DC-DC.
// --------------------------------------------------------
#define DCDCCTL_DISABLE   0   // Disables DC-DC and the device would work of internal VREG.
#define DCDCCTL_ENABLE    1   // Enables DC-DC.

// CMPHPMXSEL Register (Offset = 82h) [reset = 0h]
// CMPLPMXSEL Register (Offset = 84h) [reset = 0h]
// CMPHNMXSEL Register (Offset = 86h) [reset = 0h]
// CMPLNMXSEL Register (Offset = 87h) [reset = 0h]
//////////////////////////////////////////////////////////////////////////

// ADCDACLOOPBACK Register (Offset = 88h) [reset = 0h]
// Enabble loopback from DAC to ADCs
//////////////////////////////////////////////////////////////////////////
// ENLB2ADCC 2 bit
// ENLB2ADCB 1 bit
// ENLB2ADCA 0 bit
// Loops back COMPDACA output to ADCx.
// --------------------------------------------------------
#define ENLB2ADCx_DISABLE   0
#define ENLB2ADCx_ENABLE    1


// 将原始的 ADC 值转换为温度值
// ---------------------------------------------
static inline int16_t GetTemperatureC(uint16_t tempResult, float vref)
{
    float temp;

    //
    // Read temp sensor slope and offset locations from OTP and convert
    //
    temp = (float)tempResult * (vref / 2.5F);
    return((int16_t)((((int32_t)temp - ADC_EXT_REF_TSOFFSET) * 4096) /
                         ADC_EXT_REF_TSSLOPE));

}

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif

//
// End of file
//
