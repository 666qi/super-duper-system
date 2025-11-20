/*
 * f28004x_x_bar.h
 *
 *  Created on: 2022年5月29日
 *      Author: jzyli
 */

#ifndef COMMON_INCLUDE_F28004X_X_BAR_H_
#define COMMON_INCLUDE_F28004X_X_BAR_H_

// TRIP4MUXENABLE - TRIP12MUXENABLE
// ePWM XBAR Mux Enable

#define EPWM_XBAR_DISABLE      0
#define EPWM_XBAR_ENABLE       1

// ePWM X-BAR Mux Configuration Table

// Mux0
#define EPWM_XBAR_Mux0_CMPSS1_CTRIPH                0
#define EPWM_XBAR_Mux0_CMPSS1_CTRIPH_OR_CTRIPL      1
#define EPWM_XBAR_Mux0_ADCAEVT1                     2
#define EPWM_XBAR_Mux0_ECAP1OUT                     3

// Mux1
#define EPWM_XBAR_Mux1_CMPSS1_CTRIPL                0
#define EPWM_XBAR_Mux1_INPUTXBAR1                   1
#define EPWM_XBAR_Mux1_CLB1_OUT4                    2
#define EPWM_XBAR_Mux1_ADCCEVT1                     3

// Mux2
#define EPWM_XBAR_Mux2_CMPSS2_CTRIPH                0
#define EPWM_XBAR_Mux2_CMPSS2_CTRIPH_OR_CTRIPL      1
#define EPWM_XBAR_Mux2_ADCAEVT2                     2
#define EPWM_XBAR_Mux2_ECAP2OUT                     3

// Mux3
#define EPWM_XBAR_Mux3_CMPSS2_CTRIPL                0
#define EPWM_XBAR_Mux3_INPUTXBAR2                   1
#define EPWM_XBAR_Mux3_CLB1_OUT5                    2
#define EPWM_XBAR_Mux3_ADCCEVT2                     3

// Mux4
#define EPWM_XBAR_Mux4_CMPSS3_CTRIPH                0
#define EPWM_XBAR_Mux4_CMPSS3_CTRIPH_OR_CTRIPL      1
#define EPWM_XBAR_Mux4_ADCAEVT3                     2
#define EPWM_XBAR_Mux4_ECAP3OUT                     3

// Mux5
#define EPWM_XBAR_Mux5_CMPSS3_CTRIPL                0
#define EPWM_XBAR_Mux5_INPUTXBAR3                   1
#define EPWM_XBAR_Mux5_CLB2_OUT4                    2
#define EPWM_XBAR_Mux5_ADCCEVT3                     3

// Mux6
#define EPWM_XBAR_Mux6_CMPSS4_CTRIPH                0
#define EPWM_XBAR_Mux6_CMPSS4_CTRIPH_OR_CTRIPL      1
#define EPWM_XBAR_Mux6_ADCAEVT4                     2
#define EPWM_XBAR_Mux6_ECAP4OUT                     3

// Mux7
#define EPWM_XBAR_Mux7_CMPSS4_CTRIPL                0
#define EPWM_XBAR_Mux7_INPUTXBAR4                   1
#define EPWM_XBAR_Mux7_CLB2_OUT5                    2
#define EPWM_XBAR_Mux7_ADCCEVT4                     3

// Mux8
#define EPWM_XBAR_Mux8_CMPSS5_CTRIPH                0
#define EPWM_XBAR_Mux8_CMPSS5_CTRIPH_OR_CTRIPL      1
#define EPWM_XBAR_Mux8_ADCBEVT1                     2
#define EPWM_XBAR_Mux8_ECAP5OUT                     3

// Mux9
#define EPWM_XBAR_Mux9_CMPSS5_CTRIPL                0
#define EPWM_XBAR_Mux9_INPUTXBAR5                   1
#define EPWM_XBAR_Mux9_CLB3_OUT4                    2


// Mux10
#define EPWM_XBAR_Mux10_CMPSS6_CTRIPH                0
#define EPWM_XBAR_Mux10_CMPSS6_CTRIPH_OR_CTRIPL      1
#define EPWM_XBAR_Mux10_ADCBEVT2                     2
#define EPWM_XBAR_Mux10_ECAP6OUT                     3

// Mux11
#define EPWM_XBAR_Mux11_CMPSS6_CTRIPL                0
#define EPWM_XBAR_Mux11_INPUTXBAR6                   1
#define EPWM_XBAR_Mux11_CLB3_OUT5                    2


// Mux12
#define EPWM_XBAR_Mux12_CMPSS7_CTRIPH                0
#define EPWM_XBAR_Mux12_CMPSS7_CTRIPH_OR_CTRIPL      1
#define EPWM_XBAR_Mux12_ADCBEVT3                     2
#define EPWM_XBAR_Mux12_ECAP7OUT                     3

// Mux13
#define EPWM_XBAR_Mux13_CMPSS7_CTRIPL                0
#define EPWM_XBAR_Mux13_ADCSOCAO                     1
#define EPWM_XBAR_Mux13_CLB4_OUT4                    2


// Mux14
#define EPWM_XBAR_Mux14_ADCBEVT4                    2
#define EPWM_XBAR_Mux14_EXTSYNCOUT                  3

// Mux15

#define EPWM_XBAR_Mux15_ADCSOCBO                   1
#define EPWM_XBAR_Mux15_CLB4_OUT5                  2


// Mux16
#define EPWM_XBAR_Mux16_SD1FLT1_COMPH                0
#define EPWM_XBAR_Mux16_SD1FLT1_COMPH_OR_COMPL       1


// Mux17
#define EPWM_XBAR_Mux17_SD1FLT1_COMPL                0
#define EPWM_XBAR_Mux17_INPUT7                       1

#define EPWM_XBAR_Mux17_CLAHALT                      3

// Mux18
#define EPWM_XBAR_Mux18_SD1FLT2_COMPH                0
#define EPWM_XBAR_Mux18_SD1FLT2_COMPH_OR_COMPL       1


// Mux19
#define EPWM_XBAR_Mux19_SD1FLT2_COMPL                0
#define EPWM_XBAR_Mux19_INPUT8                       1


// Mux20
#define EPWM_XBAR_Mux20_SD1FLT3_COMPH                0
#define EPWM_XBAR_Mux20_SD1FLT3_COMPH_OR_COMPL       1


// Mux21
#define EPWM_XBAR_Mux21_SD1FLT3_COMPL                0
#define EPWM_XBAR_Mux21_INPUT9                       1

// Mux22
#define EPWM_XBAR_Mux22_SD1FLT4_COMPH                0
#define EPWM_XBAR_Mux22_SD1FLT4_COMPH_OR_COMPL       1

// Mux23
#define EPWM_XBAR_Mux23_SD1FLT4_COMPL                0
#define EPWM_XBAR_Mux23_INPUT10                      1

// INPUT1-16SELECT Register
#define INPUT_XBAR_GPIO0      0
#define INPUT_XBAR_GPIO1      1
#define INPUT_XBAR_GPIO2      2
#define INPUT_XBAR_GPIO3      3
#define INPUT_XBAR_GPIO4      4
#define INPUT_XBAR_GPIO5      5
#define INPUT_XBAR_GPIO6      6
#define INPUT_XBAR_GPIO7      7
#define INPUT_XBAR_GPIO8      8
#define INPUT_XBAR_GPIO9      9
#define INPUT_XBAR_GPIO10     10
#define INPUT_XBAR_GPIO11     11
#define INPUT_XBAR_GPIO12     12
#define INPUT_XBAR_GPIO13     13
#define INPUT_XBAR_GPIO14     14
#define INPUT_XBAR_GPIO15     15
#define INPUT_XBAR_GPIO16     16
#define INPUT_XBAR_GPIO17     17
#define INPUT_XBAR_GPIO18     18
#define INPUT_XBAR_GPIO19     19
#define INPUT_XBAR_GPIO20     20
#define INPUT_XBAR_GPIO21     21
#define INPUT_XBAR_GPIO22     22
#define INPUT_XBAR_GPIO23     23
#define INPUT_XBAR_GPIO24     24
#define INPUT_XBAR_GPIO25     25
#define INPUT_XBAR_GPIO26     26
#define INPUT_XBAR_GPIO27     27
#define INPUT_XBAR_GPIO28     28
#define INPUT_XBAR_GPIO29     29
#define INPUT_XBAR_GPIO30     30
#define INPUT_XBAR_GPIO31     31
#define INPUT_XBAR_GPIO32     32
#define INPUT_XBAR_GPIO33     33
#define INPUT_XBAR_GPIO34     34
#define INPUT_XBAR_GPIO35     35
#define INPUT_XBAR_GPIO36     36

// 自己继续添加
// #define INPUT_XBAR_GPIO37     37

// INPUTSELECTLOCK
// Input Select Lock Register
// 0: Register is not locked
// 1: Register is locked
#define INPUT_XBAR_UNLOCK    0
#define INPUT_XBAR_LOCK      1


#endif /* COMMON_INCLUDE_F28004X_X_BAR_H_ */
