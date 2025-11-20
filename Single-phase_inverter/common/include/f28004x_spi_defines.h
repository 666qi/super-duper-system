/*
 * f28004x_spi_defines.h
 *
 *  Created on: 2022Äê6ÔÂ6ÈÕ
 *      Author: jzyli
 */

#ifndef COMMON_INCLUDE_F28004X_SPI_DEFINES_H_
#define COMMON_INCLUDE_F28004X_SPI_DEFINES_H_

/*----------------------------------------------------------------------------------------------------------------
 * 0h SPICCR SPI Configuration Control Register
 * 1h SPICTL SPI Operation Control Register
 * 2h SPISTS SPI Status Register
 * 4h SPIBRR SPI Baud Rate Register
 * 6h SPIRXEMU SPI Emulation Buffer Register
 * 7h SPIRXBUF SPI Serial Input Buffer Register
 * 8h SPITXBUF SPI Serial Output Buffer Register
 * 9h SPIDAT SPI Serial Data Register
 * Ah SPIFFTX SPI FIFO Transmit Register
 * Bh SPIFFRX SPI FIFO Receive Register
 * Ch SPIFFCT SPI FIFO Control Register
 * Fh SPIPRI SPI Priority Control Register
----------------------------------------------------------------------------------------------------------------*/


// SPIFFTX Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SPIFFTX contains both control and status bits related to the output FIFO buffer.
 * This includes FIFO reset control, FIFO interrupt level control, FIFO level status,
 * as well as FIFO interrupt enable and clear bits.
 * ----------------------------------------------------------------------------------------------------------------*/
// 15 BIT SPIRST
// SPI Reset
#define SPIFFTX_SPIRST_RESET      0x0
#define SPIFFTX_SPIRST_RESUME     0x1

// 14 BIT SPIFFENA
// SPI FIFO enable
#define SPIFFTX_SPIFFENA_DISABLE      0x0
#define SPIFFTX_SPIFFENA_ENABLE       0x1

// 13 BIT TXFIFO
//Transmit FIFO reset
#define SPIFFTX_TXFIFO_HOLDRESET      0x0
#define SPIFFTX_TXFIFO_RELEASE        0x1

// 12-8 BIT TXFFST
// FIFO status read-only

// 7 BIT TXFFINT
// Transmit FIFO interrupt read-only

// 6 BIT TXFFINTCLR
// Transmit FIFO interrupt clear
#define SPIFFTX_TXFFINTCLR_NOEFFECT    0x0
#define SPIFFTX_TXFFINTCLR_CLEAR       0x1

// 5 BIT TXFFIENA
// Transmit FIFO interrupt enable
#define SPIFFTX_TXFFIENA_DISABLE      0x0
#define SPIFFTX_TXFFIENA_ENABLE       0x1

// 4-0 BIT TXFFIL
// TXFFIL4-0 Transmit FIFO interrupt level bits.

// SPIFFRX Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SPIFFRX contains both control and status bits related to the input FIFO buffer.
 * This includes FIFO reset control, FIFO interrupt level control, FIFO level status,
 * as well as FIFO interrupt enable and clear bits.
 * ----------------------------------------------------------------------------------------------------------------*/
// 15 BIT RXFFOVF
// Receive FIFO overflow read-only

// 14 BIT RXFFOVRCLR
// RXFFOVF clear
#define SPIFFRX_RXFFOVRCLR_NOEFFECT      0x0
#define SPIFFRX_RXFFOVRCLR_CLEAR         0x1

// 13 BIT RXFIFORESET
// Receive FIFO reset
#define SPIFFRX_RXFIFORESET_HOLDRESET      0x0
#define SPIFFRX_RXFIFORESET_RELEASE        0x1

// 12-8 BIT RXFFST
// FIFO status read-only

// 7 BIT RXFFINT
// Receive FIFO interrupt read-only

// 6 BIT RXFFINTCLR
// Receive FIFO interrupt clear
#define SPIFFRX_RXFFINTCLR_NOEFFECT    0x0
#define SPIFFRX_RXFFINTCLR_CLEAR       0x1

// 5 BIT RXFFIENA
// Receive FIFO interrupt enable
#define SPIFFRX_RXFFIENA_DISABLE      0x0
#define SPIFFRX_RXFFIENA_ENABLE       0x1

// 4-0 BIT RXFFIL
// Receive FIFO interrupt level bits

// SPIFFCT Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SPIFFCT controls the FIFO transmit delay bits.
 *
 * ----------------------------------------------------------------------------------------------------------------*/
// 15-8 BIT RESERVED

// 7-0  BIT TXDLY
// FIFO transfer delay

//  SPICCR Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SPICCR controls the setup of the SPI for operation.
 *
 * ----------------------------------------------------------------------------------------------------------------*/
// 15-8 BIT RESERVED
// 7 BIT STOPBITS
// SPI software reset (active low)
#define SPICCR_SPISWRESET_RESET       0x0
#define SPICCR_SPISWRESET_REENABLED   0x1

// 6 BIT CLKPOLARITY
// Shift Clock Polarity
// This bit controls the polarity of the SPICLK signal.
#define SPICCR_CLKPOLARITY_0    0x0
#define SPICCR_CLKPOLARITY_1    0x1

// 5 BIT HS_MODE
// High Speed Mode Enable Bits
#define SPICCR_HS_MODE_DISABLED     0x0
#define SPICCR_HS_MODE_ENABLED      0x1

// 4 BIT SPILBK
// SPI Loopback Mode Select
#define SPICCR_LOOPBKENA_DISABLED  0x0
#define SPICCR_LOOPBKENA_ENABLED   0x1

// 3-0 BITS SPICHAR
// Character Length Control Bits
#define SPICCR_SPICHAR_LENGTH_1      0x0
#define SPICCR_SPICHAR_LENGTH_2      0x1
#define SPICCR_SPICHAR_LENGTH_3      0x2
#define SPICCR_SPICHAR_LENGTH_4      0x3
#define SPICCR_SPICHAR_LENGTH_5      0x4
#define SPICCR_SPICHAR_LENGTH_6      0x5
#define SPICCR_SPICHAR_LENGTH_7      0x6
#define SPICCR_SPICHAR_LENGTH_8      0x7
#define SPICCR_SPICHAR_LENGTH_9      0x8
#define SPICCR_SPICHAR_LENGTH_10      0x9
#define SPICCR_SPICHAR_LENGTH_11      0xA
#define SPICCR_SPICHAR_LENGTH_12      0xB
#define SPICCR_SPICHAR_LENGTH_13      0xC
#define SPICCR_SPICHAR_LENGTH_14      0xD
#define SPICCR_SPICHAR_LENGTH_15      0xE
#define SPICCR_SPICHAR_LENGTH_16      0xF

//  SPICTL Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SPICTL controls data transmission, the SPI's ability to generate interrupts,
 * the SPICLK phase, and the operational mode (slave or master).
 *
 *------------------------------------------------------------------------------------------------------------------*/
// 15-5 RESERVED
// 4 BIT OVERRUNINTENA
// Overrun Interrupt Enable
#define SPICTL_OVERRUNINTENA_DISABLED  0x0
#define SPICTL_OVERRUNINTENA_ENABLED   0x1

// 3 CLK_PHASE
// SPI Clock Phase Select
#define SPICTL_CLK_PHASE_NORMAL       0x0
#define SPICTL_CLK_PHASE_DELAY        0x1

// 2 BIT MASTER_SLAVE
// SPI Network Mode Control
#define SPICTL_SLAVE                0x0
#define SPICTL_MASTER               0x1

// 1 BIT TALK
// Transmit Enable
#define SPICTL_TALK_DISABLED  0x0
#define SPICTL_TALK_ENABLED   0x1

// 0 SPIINTENA
// SPI Interrupt Enable
#define SPICTL_SPIINTENA_DISABLED  0x0
#define SPICTL_SPIINTENA_ENABLED   0x1


//  SPIBRR Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SPIBRR contains the bits used for baud-rate selection.
 *
 *------------------------------------------------------------------------------------------------------------------*/
// 15-7 RESERVED

// 6-0  SPI_BIT_RATE
// SPI Baud Rate Control


// SPIRXBUF Register Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SPIRXBUF contains the received data.
 *------------------------------------------------------------------------------------------------------------------*/
// 15-0 BIT RXBn
//

//  SPITXBUF Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SPITXBUF stores the next character to be tranmitted.
 *------------------------------------------------------------------------------------------------------------------*/
// 15-0 BIT TXBn
//

// SPIPRI Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SPIPRI controls auxillary functions for the SPI including emulation control,
 * SPISTE inversion, and 3-wire control.
 *------------------------------------------------------------------------------------------------------------------*/
// 15-6 BIT RESERVED
// 5 BIT SOFT
#define SPIPRI_SOFT_0        0x0
#define SPIPRI_SOFT_1        0x1
// 4 BIT FREE
#define SPIPRI_FREE_EMULATION  0x0
#define SPIPRI_FREE_RUN        0x1
// 1 BIT STEINV
#define SPIPRI_STEINV_NOMAL   0x0
#define SPIPRI_STEINV_INVERT  0x1
// 0 BIT TRIWIRE
#define SPIPRI_TRIWIRE_4   0x0
#define SPIPRI_TRIWIRE_3   0x1

#endif /* COMMON_INCLUDE_F28004X_SPI_DEFINE_H_ */
