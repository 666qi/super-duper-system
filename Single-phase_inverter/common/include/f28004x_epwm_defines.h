//###########################################################################
//
// FILE:   f28004x_epwm_defines.h
//
// TITLE:  f28004x Device EPWM Register Bit Defines
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

#ifndef F28004x_EPWM_DEFINES_H
#define F28004x_EPWM_DEFINES_H

#ifdef __cplusplus
extern "C" {
#endif

//
// Defines
//

//
// TBCTL (Time-Base Control)
////////////////////////////////////////////////////////////////

//
// 15-14 FREE_SOFT Emulation Mode
#define TB_NEXT        0x0
#define TB_COMPLETES   0x1
#define TB_FREE        0x2

// 13 PHSDIR
// In the up-count and down-count modes this bit is ignored.
// This bit is only used when the time-base counter is configured in the up-down-count mode.
#define TB_PHSDIR_DOWN     0x0
#define TB_PHSDIR_UP       0x1

//
// 12-10 CLKDIV bits
//
#define TB_CLKDIV1     0x0 // 000: /1 (default on reset)
#define TB_CLKDIV2     0x1
#define TB_CLKDIV4     0x2
#define TB_CLKDIV8     0x3
#define TB_CLKDIV16    0x4
#define TB_CLKDIV32    0x5
#define TB_CLKDIV64    0x6
#define TB_CLKDIV128   0x7

//
// 9-7 HSPCLKDIV bits
//
#define TB_HSPCLKDIV1    0x0 // 000: /1 (default on reset)
#define TB_HSPCLKDIV2    0x1
#define TB_HSPCLKDIV4    0x2
#define TB_HSPCLKDIV6    0x3
#define TB_HSPCLKDIV8    0x4
#define TB_HSPCLKDIV10   0x5
#define TB_HSPCLKDIV12   0x6
#define TB_HSPCLKDIV14   0x7

// 6 SWFSYNC bits
//
#define TB_SWFSYNC_DISABLE     0x0
#define TB_SWFSYNC_ENABLE      0x1

// 5-4 SYNCOSEL bits
//
#define TB_EPWMxSYNCI_SWFSYNC       0x0
#define TB_CTR_ZERO                 0x1
#define TB_CTR_CMPB                 0x2
#define TB_TBCTL2_SYNCOSELX         0x3

//
// 3 PRDLD bit
//
#define TB_SHADOW     0x0
#define TB_IMMEDIATE  0x1
//
// 2 PHSEN bit
//
#define TB_PHSEN_DISABLE  0x0
#define TB_PHSEN_ENABLE   0x1

// 1-0 CTRMODE bits
//
#define TB_COUNT_UP      0x0
#define TB_COUNT_DOWN    0x1
#define TB_COUNT_UPDOWN  0x2
#define TB_FREEZE        0x3

//
// CMPCTL (Compare Control)
///////////////////////////////////////////////////////////////////

//
// LOADAMODE and LOADBMODE bits
//
#define CC_CTR_ZERO      0x0
#define CC_CTR_PRD       0x1
#define CC_CTR_ZERO_PRD  0x2
#define CC_LD_DISABLE    0x3

//
// SHDWAMODE and SHDWBMODE bits
//
#define CC_SHADOW     0x0
#define CC_IMMEDIATE  0x1

// AQCTL
// Action Qualifier Control Register
//

//
// LDAQASYNC and LDAQBSYNC bits
//
#define AQ_LOADMODE              0x0
#define AQ_LOADMODEANDSYNCEVENT  0x1
#define AQ_SYNCEVENT             0x2
//
// SHDWAQAMODE and SHDWAQBMODE bits
//
#define AQ_IMMEDIATE  0x0
#define AQ_SHADOW     0x1

//
// LOADAMODE and LOADBMODE bits
//
#define AQ_CTR_ZERO      0x0
#define AQ_CTR_PRD       0x1
#define AQ_CTR_ZERO_PRD  0x2
#define AQ_LD_DISABLE    0x3

//
// AQCTLA and AQCTLB (Action Qualifier Control)
/////////////////////////////////////////////////////////////////////

// ZRO, PRD, CAU, CAD, CBU, CBD bits
//
#define AQ_NO_ACTION  0x0
#define AQ_CLEAR      0x1
#define AQ_SET        0x2
#define AQ_TOGGLE     0x3

//
// DBCTL (Dead-Band Control)
/////////////////////////////////////////////////////////////////////

// 15 Bit HALFCYCLE
#define DB_Fullcycle    0x0
#define DB_Halfcycle    0x1
//
// DEDB_MODE
//
#define B_Mode_DISABLE   0x0  // S8 = 0 ，下降沿和上升沿通道由 S5 S4决定，输出到A B两个信号路径。
#define B_Mode_ENABLE    0x1  // S8 = 1 ，下降沿和上升沿通道由 S4决定 ，并仅输出到B信号路径。
//
// OUT MODE bits
//
#define DB_DISABLE      0x0
#define DBB_ENABLE      0x1
#define DBA_ENABLE      0x2
#define DB_FULL_ENABLE  0x3

//
// 3-2 bit POLSEL bits
//
// bit3 控制 S3 开关，bit2 控制 S2 开关
// S3 开关下降沿
// S2 开关上升沿
// EPWMxA and EPWMxB在输出之前可以选择反转其中一个延迟信号
#define DB_ACTV_HI   0x0  // EPWMxA and EPWMxB 都不反极
#define DB_ACTV_LOC  0x1  // EPWMxA is inverted
#define DB_ACTV_HIC  0x2  // EPWMxB is inverted
#define DB_ACTV_LO   0x3  // EPWMxA and EPWMxB 都反极

//
// IN MODE bits
//
#define DBA_ALL          0x0
#define DBB_RED_DBA_FED  0x1
#define DBA_RED_DBB_FED  0x2
#define DBB_ALL          0x3

//
// OUTSWAP bits
//
#define OUTSWAP_INVALID          0x0
#define OUTSWAP_EPWMxB           0x1
#define OUTSWAP_EPWMxA           0x2
#define OUTSWAP_ALL              0x3

//
// CHPCTL (chopper control)
/////////////////////////////////////////////////////////////////////

//
// CHPEN bit
//
#define CHP_DISABLE  0x0
#define CHP_ENABLE   0x1

//
// CHPFREQ bits
//
#define CHP_DIV1  0x0
#define CHP_DIV2  0x1
#define CHP_DIV3  0x2
#define CHP_DIV4  0x3
#define CHP_DIV5  0x4
#define CHP_DIV6  0x5
#define CHP_DIV7  0x6
#define CHP_DIV8  0x7

//
// CHPDUTY bits
//
#define CHP1_8TH  0x0
#define CHP2_8TH  0x1
#define CHP3_8TH  0x2
#define CHP4_8TH  0x3
#define CHP5_8TH  0x4
#define CHP6_8TH  0x5
#define CHP7_8TH  0x6

//
// TZSEL (Trip Zone Select)
/////////////////////////////////////////////////////////////////////

//
// CBCn and OSHTn bits
//
#define TZ_DCBEVT1_DISABLE  0x0
#define TZ_DCBEVT1_ENABLE   0x1

#define TZ_DCAEVT1_DISABLE  0x0
#define TZ_DCAEVT1_ENABLE   0x1

#define TZ_DCBEVT2_DISABLE  0x0
#define TZ_DCBEVT2_ENABLE   0x1

#define TZ_DCAEVT2_DISABLE  0x0
#define TZ_DCAEVT2_ENABLE   0x1

#define TZ_CBC6_DISABLE  0x0
#define TZ_CBC6_ENABLE   0x1

#define TZ_CBC5_DISABLE  0x0
#define TZ_CBC5_ENABLE   0x1

#define TZ_CBC4_DISABLE  0x0
#define TZ_CBC4_ENABLE   0x1

#define TZ_CBC3_DISABLE  0x0
#define TZ_CBC3_ENABLE   0x1

#define TZ_CBC2_DISABLE  0x0
#define TZ_CBC2_ENABLE   0x1

#define TZ_CBC1_DISABLE  0x0
#define TZ_CBC1_ENABLE   0x1

#define TZ_OSHT6_DISABLE  0x0
#define TZ_OSHT6_ENABLE   0x1

#define TZ_OSHT5_DISABLE  0x0
#define TZ_OSHT5_ENABLE   0x1

#define TZ_OSHT4_DISABLE  0x0
#define TZ_OSHT4_ENABLE   0x1

#define TZ_OSHT3_DISABLE  0x0
#define TZ_OSHT3_ENABLE   0x1

#define TZ_OSHT2_DISABLE  0x0
#define TZ_OSHT2_ENABLE   0x1

#define TZ_OSHT1_DISABLE  0x0
#define TZ_OSHT1_ENABLE   0x1
//
// TZCTL (Trip Zone Control)
//

//
// TZA and TZB bits
//
#define TZ_HIZ       0x0
#define TZ_FORCE_HI  0x1
#define TZ_FORCE_LO  0x2
#define TZ_NO_CHANGE 0x3


// TZFRC Register
// Trip Zone Force Register
// 0: Writing 0 has no effect. This bit always reads back 0.
//--------------------------------------------------------------
// 6bit DCBEVT2
#define TZ_TZFRC_DCBEVT2      0x1 // 1: Writing 1 forces the DCBEVT2 event trip condition and sets the TZFLG[DCBEVT2] bit.
// 5bit DCBEVT1
#define TZ_TZFRC_DCBEVT1      0x1 // 1: Writing 1 forces the DCBEVT1 event trip condition and sets the TZFLG[DCBEVT1] bit.
// 4bit DCAEVT2
#define TZ_TZFRC_DCAEVT2      0x1 // 1: Writing 1 forces the DCAEVT2 event trip condition and sets the TZFLG[DCAEVT2] bit.
// 3bit DCAEVT1
#define TZ_TZFRC_DCAEVT1      0x1 // 1: Writing 1 forces the DCAEVT1 event trip condition and sets the TZFLG[DCAEVT1] bit.
// 2bit OST
#define TZ_TZFRC_OST          0x1 // 1: Forces a one-shot trip event and sets the TZFLG[OST] bit.
// 1bit DCAEVT1
#define TZ_TZFRC_CBC          0x1 // 1: Forces a cycle-by-cycle trip event and sets the TZFLG[CBC] bit.

//
// TZDCSEL (Trip Zone Digital Compare)
//

//
// DCAEVT1, DCAEVT2, DCBEVT1, DCBEVT2 bits
//
#define TZ_EVT_DISABLE      0x0
#define TZ_DCAH_LOW         0x1
#define TZ_DCAH_HI          0x2
#define TZ_DCAL_LOW         0x3
#define TZ_DCAL_HI          0x4
#define TZ_DCAL_HI_DCAH_LOW 0x5

#define TZ_DCBH_LOW         0x1
#define TZ_DCBH_HI          0x2
#define TZ_DCBL_LOW         0x3
#define TZ_DCBL_HI          0x4
#define TZ_DCBL_HI_DCBH_LOW 0x5


//
// TZCLR
// Trip Zone Clear Register
////////////////////////////////////////////////////////////

// CBCPULSE bits
#define TZ_CBCPULSE_ZERO         0x0
#define TZ_CBCPULSE_PRD          0x1
#define TZ_CBCPULSE_ZERO_PRD     0x3
#define TZ_CBCPULSE_INVALID      0x4

// DCBEVT2 bit
#define TZ_DCBEVT2_INVALID         0x0
#define TZ_DCBEVT2_CLEAR           0x1

// DCBEVT1 bit
#define TZ_DCBEVT1_INVALID         0x0
#define TZ_DCBEVT1_CLEAR           0x1

// DCAEVT2 bit
#define TZ_DCAEVT2_INVALID         0x0
#define TZ_DCAEVT2_CLEAR           0x1

// DCAEVT1 bit
#define TZ_DCAEVT1_INVALID         0x0
#define TZ_DCAEVT1_CLEAR           0x1

// OST bit
#define TZ_OST_INVALID         0x0
#define TZ_OST_CLEAR           0x1

// CBC bit
#define TZ_CBC_INVALID         0x0
#define TZ_CBC_CLEAR           0x1

// INT BIT
#define TZ_INT_INVALID         0x0
#define TZ_INT_CLEAR           0x1

//
// ETSEL Register
//
////////////////////////////////////////////////////////////

// INTEN
#define EPWMxINT_DISABLE  0  // Disable EPWMx_INT generation
#define EPWMxINT_ENABLE   1  // Enable EPWMx_INT generation

//
// SOCAEN and SOCBEN bit
//
#define EPWMxSOC_DISABLE  0
#define EPWMxSOC_ENABLE   1
//
// SOCASEL and SOCBSEL bit
//
#define ET_DCAEVT1SOC    0x0
#define ET_CTR_ZERO      0x1
#define ET_CTR_PRD       0x2
#define ET_CTR_PRDZERO   0x3
#define ET_CTRU_CMPAC    0x4
#define ET_CTRD_CMPAC    0x5
#define ET_CTRU_CMPBD    0x6
#define ET_CTRD_CMPBD    0x7
//
// INTSELCMP and SOCASELCMP and SOCBSELCMP bit

// EPWMxINT Compare Register Selection Options
// EPWMxSOCB Compare Register Selection Options
// EPWMxSOCA Compare Register Selection Options
#define ET_SELCMP_AB   0x0
#define ET_SELCMP_CD   0x1



//
// ETPS (Event Trigger Pre-scale)
//
//////////////////////////////////////////////////////////////////

//
// SOCPSSEL and INTPSSEL bit
//
#define ET_PSSEL_3    0x0
#define ET_PSSEL_15   0x1
//
// INTPRD, SOCAPRD, SOCBPRD bits
//
#define ET_DISABLE  0x0
#define ET_1ST      0x1
#define ET_2ND      0x2
#define ET_3RD      0x3

//
// ETSOCPS
// Event-Trigger SOC Pre-Scale Register
//
///////////////////////////////////////////////////////////////////
//
// SOCAPRD2, SOCBPRD2 bits
//
#define ET_DISABLE   0x0
#define ET_1ST       0x1
#define ET_2ND       0x2
#define ET_3RD       0x3
#define ET_4TH       0x4
#define ET_5TH       0x5
#define ET_6TH       0x6
#define ET_7TH       0x7
#define ET_8TH       0x8
#define ET_9TH       0x9
#define ET_10TH      0xA
#define ET_11TH      0xB
#define ET_12TH      0xC
#define ET_13TH      0xD
#define ET_14TH      0xE
#define ET_15TH      0xF

// ETCLR Register (Offset = AAh) [reset = 0h]
#define ETCLR_INVALID    0
#define ETCLR_CLEAR      1

// ETCNTINITCTL Register (Offset = B2h) [reset = 0h]
// ------------------------------------------------------------------
// 15 SOCBINITEN  EPWMxSOCB Counter 2 Initialization Enable
// 14 SOCAINITEN  EPWMxSOCA Counter 2 Initialization Enable
// 13 INTINITEN   EPWMxINT Counter 2 Initialization Enable
// 12 SOCBINITFRC EPWMxSOCB Counter 2 Initialization Force
// 11 SOCAINITFRC EPWMxSOCA Counter 2 Initialization Force
// 10 INTINITFRC  EPWMxINT Counter 2 Initialization Force
#define ETCNTINITCTL_Init      1


// HRPWM (High Resolution PWM)
//

//
// HRCNFG
//
#define HR_AUTOCONV_DISABLE   0x0
#define HR_AUTOCONV_ENABLE    0x1

#define HR_DISABLE        0x0  // HRPWM capability is disabled (default on reset)
#define HR_REP            0x1  // MEP control of rising edge (CMPAHR)
#define HR_FEP            0x2  // MEP control of falling edge (CMPAHR)
#define HR_BEP            0x3  // MEP control of both edges (TBPHSHR or TBPRDHR)

#define HR_CMP            0x0  // CMPAHR(8) or TBPRDHR(8) Register controls the edge position
#define HR_PHS            0x1  // TBPHSHR(8) Register controls the edge position

#define HR_CTR_ZERO       0x0
#define HR_CTR_PRD        0x1
#define HR_CTR_ZERO_PRD   0x2

#define HR_NORM_B         0x0
#define HR_INVERT_B       0x1

//
// HRPCTL
//
// PWMSYNCSELX
// Extended selection bits for EPWMSYNCPER
// 000: EPWMSYNCPER is defined by PWMSYNCSEL - > default condition (compatible with previous EPWM versions)
// 100: CTR = CMPC, Count direction Up
// 101: CTR = CMPC, Count direction Down
// 110: CTR = CMPD, Count direction Up
// 111: CTR = CMPD, Count direction Down

#define HR_PWMSYNCSELX_default       0x0
#define HR_PWMSYNCSELX_CMPCUP        0x4
#define HR_PWMSYNCSELX_CMPCDOWN      0x5
#define HR_PWMSYNCSELX_CMPDUP        0x6
#define HR_PWMSYNCSELX_CMPDDOWN      0x7

// TBPHSHRLOADE
// TBPHSHR Load Enable
// This bit allows you to synchronize ePWM modules with a highresolution phase on a SYNCIN, TBCTL SWFSYNC or digital compare event.
// 0: Disables synchronization of high-resolution phase on a SYNCIN,TBCTL SWFSYNC or digital compare event:
// 1: Synchronize the high-resolution phase on a SYNCIN, TBCTL SWFSYNC or digital comparator synchronization event.

#define HR_TBPHSHRLOADE_DISABLE      0x0
#define HR_TBPHSHRLOADE_ENABLE       0x1

// PWMSYNCSEL
// PWMSYNC Source Select Bit
// This bit selects the source for the EPWMSYNCPER signal that goes to the CMPSS and GPDAC:
// 0 CTR = PRD: Time-base counter equal to period (TBCTR = TBPRD)
// 1 CTR = zero: Time-base counter equal to zero (TBCTR = 0x00)

#define HR_PWMSYNCSEL_PRD      0x0
#define HR_PWMSYNCSEL_ZERO     0x1

// HRPE
// High Resolution Period Enable Bit
// 0: High resolution period feature disabled.
// 1: High resolution period enabled.

#define HR_HRPE_DISABLE      0x0
#define HR_HRPE_ENABLE       0x1

//
// DC (Digital Compare)
//

//
// DCAHTRIPSEL DCALTRIPSEL
// DCBHTRIPSEL DCBLTRIPSEL
#define TRIPSEL_COMBINATION_DISABLE           0x0
#define TRIPSEL_COMBINATION_ENABLE            0x1
//
// DCTRIPSEL
//
#define DC_TZ1           0x0
#define DC_TZ2           0x1
#define DC_TZ3           0x2
#define DC_TRIPIN1       0x0
#define DC_TRIPIN2       0x1
#define DC_TRIPIN3       0x2
#define DC_TRIPIN4       0x3
#define DC_TRIPIN5       0x4
#define DC_TRIPIN6       0x5
#define DC_TRIPIN7       0x6
#define DC_TRIPIN8       0x7
#define DC_TRIPIN9       0x8
#define DC_TRIPIN10      0x9
#define DC_TRIPIN11      0xA
#define DC_TRIPIN12      0xB
//      Reserved         0xC
#define DC_TRIPIN14      0xD
#define DC_TRIPIN15      0xE
#define DC_COMBINATION   0xF

//
// DCFCTL
//
#define DC_SRC_DCAEVT1   0x0
#define DC_SRC_DCAEVT2   0x1
#define DC_SRC_DCBEVT1   0x2
#define DC_SRC_DCBEVT2   0x3

#define DC_PULSESEL_PRD      0x0
#define DC_PULSESEL_ZERO     0x1
#define DC_PULSESEL_ZERO_PRD 0x2

#define DC_BLANK_DISABLE 0x0
#define DC_BLANK_ENABLE  0x1

#define DC_BLANK_NOTINV  0x0
#define DC_BLANK_INV     0x1

//
//DCACTL/DCBCTL
//
#define DC_EVT1          0x0
#define DC_EVT2          0x0
#define DC_EVT_FILT      0x1

#define DC_EVT2FRCSYNCSEL_ASYNC          0x0
#define DC_EVT2FRCSYNCSEL_EPWMCLK        0x1

#define DC_EVT1FRCSYNCSEL_EPWMCLK      0x0
#define DC_EVT1FRCSYNCSEL_ASYNC        0x1

#define DC_EVT1SOCE_DISABLE   0x0
#define DC_EVT1SOCE_ENABLE    0x1

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   // - end of F28004x_EPWM_DEFINES_H

//
// End of file
//
