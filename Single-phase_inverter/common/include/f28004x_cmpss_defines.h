
/*
 * 自己定义的
 *
*/

#ifndef F28004X_CMPSS_DEFINES_H
#define F28004X_CMPSS_DEFINES_H

#ifdef __cplusplus
extern "C" {
#endif

//
// Defines
//

//
// COMPCTL
// CMPSS Comparator Control Register

//
// COMPDACE bits
//
#define DAC_DISABLE      0x0
#define DAC_ENABLE       0x1

//
// ASYNCLEN OR ASYNCHEN bits
//
#define ASYNC_DISABLE      0x0
#define ASYNC_ENABLE       0x1

//
// CTRIPOUTLSEL OR CTRIPOUTHSEL bits
// connected to the Output X-BAR for external signaling
#define CTRIPOUT_ASYNC      0x0
#define CTRIPOUT_SYNC       0x1
#define CTRIPOUT_FILTER     0x2
#define CTRIPOUT_LATCH      0x3

//
// CTRIPLSEL OR CTRIPHSEL bits
// connected to the ePWM X-BAR for ePWM trip response
#define CTRIP_ASYNC      0x0
#define CTRIP_SYNC       0x1
#define CTRIP_FILTER     0x2
#define CTRIP_LATCH      0x3

//
// COMPLINV OR COMPHINV bits
//
#define INV_DISABLE      0x0
#define INV_ENABLE       0x1

//
// COMPLSOURCE OR COMPHSOURCE bits
//
#define NEGSOURCE_DAC      0x0
#define NEGSOURCE_PIN      0x1

//
// COMPHYSCTL Register
// CMPSS Comparator Hysteresis Control Register

//
// COMPHYS bits
//
#define HYS_INVALID       0x0
#define HYS_1TYPICAL      0x1
#define HYS_2TYPICAL      0x2
#define HYS_3TYPICAL      0x3
#define HYS_4TYPICAL      0x4

//
// COMPSTS Register
// CMPSS Comparator Status Register

//
// COMPSTSCLR Register
// CMPSS Comparator Status Clear Register


//
// COMPDACCTL Register
// CMPSS DAC Control Register
//
///////////////////////////////////////////////////////////////////
// FREESOFT bits
//
//
// BLANKEN bits
//
#define  EPWMBLANK_DISABLE          0x0
#define  EPWMBLANK_ENABLE           0x1
//
// BLANKSOURCE bits
//
#define BLANKSOURCE_EPWM1BLANK      0x0
#define BLANKSOURCE_EPWM2BLANK      0x1
#define BLANKSOURCE_EPWM3BLANK      0x2
#define BLANKSOURCE_EPWM4BLANK      0x3
#define BLANKSOURCE_EPWM5BLANK      0x4
#define BLANKSOURCE_EPWM6BLANK      0x5
#define BLANKSOURCE_EPWM7BLANK      0x6
#define BLANKSOURCE_EPWM8BLANK      0x7
#define BLANKSOURCE_EPWM9BLANK      0x8
#define BLANKSOURCE_EPWM10BLANK      0x9
#define BLANKSOURCE_EPWM11BLANK      0xA
#define BLANKSOURCE_EPWM12BLANK      0xB
#define BLANKSOURCE_EPWM13BLANK      0xC
#define BLANKSOURCE_EPWM14BLANK      0xD
#define BLANKSOURCE_EPWM15BLANK      0xE
#define BLANKSOURCE_EPWM16BLANK      0xF
//
// SWLOADSEL bits
//
#define SWLOAD_SYSCLK          0x0
#define SWLOAD_EPWMSYNCPER     0x1
//
// RAMPLOADSEL bits
//
#define RAMPLOAD_RAMPMAXREFA     0x0
#define RAMPLOAD_RAMPMAXREFS     0x1
//
// SELREF bits
//
#define DAC_REF_VDDA     0x0
#define DAC_REF_VDAC     0x1
//
// RAMPSOURCE bits
//
//
#define RAMPSOURCE_EPWM1SYNCPER      0x0
#define RAMPSOURCE_EPWM2SYNCPER      0x1
#define RAMPSOURCE_EPWM3SYNCPER      0x2
#define RAMPSOURCE_EPWM4SYNCPER      0x3
#define RAMPSOURCE_EPWM5SYNCPER      0x4
#define RAMPSOURCE_EPWM6SYNCPER      0x5
#define RAMPSOURCE_EPWM7SYNCPER      0x6
#define RAMPSOURCE_EPWM8SYNCPER      0x7
#define RAMPSOURCE_EPWM9SYNCPER      0x8
#define RAMPSOURCE_EPWM10SYNCPER      0x9
#define RAMPSOURCE_EPWM11SYNCPER      0xA
#define RAMPSOURCE_EPWM12SYNCPER      0xB
#define RAMPSOURCE_EPWM13SYNCPER      0xC
#define RAMPSOURCE_EPWM14SYNCPER      0xD
#define RAMPSOURCE_EPWM15SYNCPER      0xE
#define RAMPSOURCE_EPWM16SYNCPER      0xF
//
// DACSOURCE bits
//
#define DACSOURCE_DACHVALS      0x0
#define DACSOURCE_RAMP          0x1

///////////////////////////////////////////////////////////////////

//
// COMPSTSCLR Register
// CMPSS Comparator Status Clear Register
#define LSYNCCLREN_DISABLE      0x0
#define LSYNCCLREN_ENABLE       0x1

#define LLATCHCLR_NORESET      0x0
#define LLATCHCLR_RESET        0x1

#define HSYNCCLREN_DISABLE      0x0
#define HSYNCCLREN_ENABLE       0x1

#define HLATCHCLR_NORESET      0x0
#define HLATCHCLR_RESET        0x1
//
// CTRIPLFILCTL Register OR CTRIPHFILCTL Register
// CTRIPL Filter Control Register OR CTRIPH Filter Control Register

//
// FILINIT bits
// filter initialization
//
#define FILINIT_NOEFFECT    0x0
#define FILINIT_INIT        0x1



#ifdef __cplusplus
}
#endif /* extern "C" */

#endif

//
// End of file
//
