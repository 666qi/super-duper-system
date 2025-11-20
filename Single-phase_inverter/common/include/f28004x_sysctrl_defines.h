//###########################################################################
//
// FILE:    f28004x_sysctrl_defines.h
//
// TITLE:   f28004x LPM support definitions
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

#ifndef F28004x_SYSCTRL_DEFINES_H
#define F28004x_SYSCTRL_DEFINES_H

//
// Defines
//
#define LPM_IDLE    0x0
#define LPM_HALT    0x2
#define LPM_HIB     0x3

// jzy 添加的定义
//  XINT_REGS Registers
// -------------------------------------------------------------------------
//  XINT1CR XINT2CR XINT3CR XINT4CR XINT5CR Register
/*
00: Interrupt is selected as negative edge triggered
01: Interrupt is selected as positive edge triggered
10: Interrupt is selected as negative edge triggered
11: Interrupt is selected as positive or negative edge triggered
 */
#define XINT_POLARITY_negative   0
#define XINT_POLARITY_positive   1
#define XINT_POLARITY_both       3

// 0: Interrupt Disabled
// 1: Interrupt Enabled
#define XINT_DISABLE       0
#define XINT_ENABLE        1



// XINT1CTR XINT2CTR XINT3CTR Register
/*
 * This is a free running 16-bit up-counter that is clocked at the SYSCLKOUT rate.
 * The counter value is reset to 0x0000 when a valid interrupt edge is detected and then continues counting until the next valid interrupt edge is detected.
 * The counter must only be reset by the selected POLARITY edge as selected in the respective interrupt control register.
 * When the interrupt is disabled, the counter will stop. The counter is a free-running counter and will wrap around to zero when the max value is reached.
 * The counter is a read only register and can only be reset to zero by a valid interrupt edge or by reset.
*/


// PCLKCR0 - PCLKCR21
#define Gate_OFF      0
#define Turn_ON       1

#endif  // end of F28004x_SYSCTRL_DEFINES_H definition

//
// End of file.
//


