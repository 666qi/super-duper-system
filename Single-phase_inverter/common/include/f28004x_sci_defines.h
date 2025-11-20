/*
 * f28004x_sci_defines.h
 *
 *  Created on: 2022Äê6ÔÂ1ÈÕ
 *      Author: jzyli
 */

#ifndef COMMON_INCLUDE_F28004X_SCI_DEFINES_H_
#define COMMON_INCLUDE_F28004X_SCI_DEFINES_H_

// SCIFFTX Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SCIFFTX controls the transmit FIFO interrupt, FIFO enhancements, and reset for the SCI transmit and receive channels.
 *
 * ----------------------------------------------------------------------------------------------------------------*/
// 15 BIT SCIRST
// SCI Reset
#define SCIFFTX_SCIRST_RESET      0x0
#define SCIFFTX_SCIRST_RESUME     0x1

// 14 BIT SCIFFENA
// SCI FIFO enable
#define SCIFFTX_SCIFFENA_DISABLE      0x0
#define SCIFFTX_SCIFFENA_ENABLE       0x1

// 13 BIT TXFIFORESET
//Transmit FIFO reset
#define SCIFFTX_TXFIFORESET_HOLDRESET      0x0
#define SCIFFTX_TXFIFORESET_REENABLE       0x1

// 12-8 BIT TXFFST
// FIFO status read-only

// 7 BIT TXFFINT
// Transmit FIFO interrupt read-only

// 6 BIT TXFFINTCLR
// Transmit FIFO interrupt clear
#define SCIFFTX_TXFFINTCLR_NOEFFECT    0x0
#define SCIFFTX_TXFFINTCLR_CLEAR       0x1

// 5 BIT TXFFIENA
// Transmit FIFO interrupt enable
#define SCIFFTX_TXFFIENA_DISABLE      0x0
#define SCIFFTX_TXFFIENA_ENABLE       0x1

// 4-0 BIT TXFFIL
// TXFFIL4-0 Transmit FIFO interrupt level bits.

// SCIFFRX Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SCIFFRX controls the receive FIFO interrupt, receive FIFO reset, and status of the receive FIFO overflow.
 *
 * ----------------------------------------------------------------------------------------------------------------*/
// 15 BIT RXFFOVF
// Receive FIFO overflow read-only

// 14 BIT RXFFOVRCLR
// RXFFOVF clear
#define SCIFFRX_RXFFOVRCLR_NOEFFECT      0x0
#define SCIFFRX_RXFFOVRCLR_CLEAR         0x1

// 13 BIT RXFIFORESET
// Receive FIFO reset
#define SCIFFRX_RXFIFORESET_HOLDRESET      0x0
#define SCIFFRX_RXFIFORESET_REENABLE       0x1

// 12-8 BIT RXFFST
// FIFO status read-only

// 7 BIT RXFFINT
// Receive FIFO interrupt read-only

// 6 BIT RXFFINTCLR
// Receive FIFO interrupt clear
#define SCIFFRX_RXFFINTCLR_NOEFFECT    0x0
#define SCIFFRX_RXFFINTCLR_CLEAR       0x1

// 5 BIT RXFFIENA
// Receive FIFO interrupt enable
#define SCIFFRX_RXFFIENA_DISABLE      0x0
#define SCIFFRX_RXFFIENA_ENABLE       0x1

// 4-0 BIT RXFFIL
// Receive FIFO interrupt level bits

// SCIFFCT Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SCIFFCT contains the status of auto-baud detect, clears the auto-baud flag, and calibrate for A-detect bit.
 *
 * ----------------------------------------------------------------------------------------------------------------*/
// 15 BIT ABD
// Auto-baud detect (ABD) bit read-only

// 14 BIT ABDCLR
// ABD-clear bit
#define SCIFFCT_ABD_NOEFFECT      0x0
#define SCIFFCT_ABD_CLEAR         0x1

// 13 BIT CDC
// CDC calibrate A-detect bit
#define SCIFFCT_CDC_DISABLE      0x0
#define SCIFFCT_CDC_ENABLE       0x1

// 12-8 BIT RESERVED

// 7-0  BIT FFTXDLY
// FIFO transfer delay

//  SCICCR Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SCICCR defines the character format, protocol, and communications mode used by the SCI.
 *
 * ----------------------------------------------------------------------------------------------------------------*/
// 15-8 BIT RESERVED
// 7 BIT STOPBITS
// SCI number of stop bits.
#define SCICCR_STOPBITS_ONE      0x0
#define SCICCR_STOPBITS_TWO      0x1

// 6 BIT PARITY
// SCI parity odd/even selection.
#define SCICCR_PARITY_ODD    0x0
#define SCICCR_PARITY_EVEN   0x1

// 5 BIT PARITYENA
// SCI parity enable
#define SCICCR_PARITYENA_DISABLED     0x0
#define SCICCR_PARITYENA_ENABLED      0x1

// 4 BIT LOOPBKENA
// Loop Back test mode enable
#define SCICCR_LOOPBKENA_DISABLED  0x0
#define SCICCR_LOOPBKENA_ENABLED   0x1

// 3 BIT ADDRIDLE_MODE
// SCI multiprocessor mode control bit
#define SCICCR_ADDRIDLE_IDLE       0x0
#define SCICCR_ADDRIDLE_ADDR       0x1

// 2-0 BITS SCICHAR
// Character-length control bits 2-0
#define SCICCR_SCICHAR_LENGTH_1      0x0
#define SCICCR_SCICHAR_LENGTH_2      0x1
#define SCICCR_SCICHAR_LENGTH_3      0x2
#define SCICCR_SCICHAR_LENGTH_4      0x3
#define SCICCR_SCICHAR_LENGTH_5      0x4
#define SCICCR_SCICHAR_LENGTH_6      0x5
#define SCICCR_SCICHAR_LENGTH_7      0x6
#define SCICCR_SCICHAR_LENGTH_8      0x7

//  SCICTL1 Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SCICTL1 controls the receiver/transmitter enable, TXWAKE and SLEEP functions, and the SCI software reset.
 *
 *------------------------------------------------------------------------------------------------------------------*/
// 15-7 RESERVED
// 6 RXERRINTENA
// SCI receive error interrupt enable
#define SCICTL1_RXERRINTENA_DISABLED  0x0
#define SCICTL1_RXERRINTENA_ENABLED   0x1

// 5 SWRESET
// SCI software reset (active low)
#define SCICTL1_SWRESET_RESET       0x0
#define SCICTL1_SWRESET_REENABLED   0x1

// 4 RESERVED
// 3 TXWAKE
// SCI transmitter wake-up method select
#define SCICTL1_TXWAKE_NOT_SELECTD                   0x0
#define SCICTL1_TXWAKE_IDLE_OR_ADDRESS               0x1

// 2 SLEEP
// SCI sleep
#define SCICTL1_SLEEP_DISABLED  0x0
#define SCICTL1_SLEEP_ENABLED   0x1

// 1 TXENA
// SCI transmitter enable
#define SCICTL1_TXENA_DISABLED  0x0
#define SCICTL1_TXENA_ENABLED   0x1

// 0 RXENA
// SCI receiver enable
#define SCICTL1_RXENA_DISABLED  0x0
#define SCICTL1_RXENA_ENABLED   0x1

//  SCICTL2 Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * SCICTL2 enables the receive-ready, break-detect, and transmit-ready interrupts as well as transmitter-ready and
 * -empty flags.
 *
 *------------------------------------------------------------------------------------------------------------------*/
// 15-8 BITS RESERVED
// 7 BIT TXRDY read-only
// Transmitter buffer register ready flag


// 6 BIT TXEMPTY read-only
// Transmitter buffer empty flag

// 5-2 BITS RESERVED

// 1 BIT RXBKINTENA
// Receiver-buffer/break interrupt enable
#define SCICTL2_RXBKINTENA_DISABLED 0x0
#define SCICTL2_RXBKINTENA_ENABLED  0x1

// 0 TXINTENA
// SCITXBUF-register interrupt enable
#define SCICTL2_TXINTENA_DISABLED   0x0
#define SCICTL2_TXINTENA_ENABLED    0x1

//  SCILBAUD Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * The values in SCIHBAUD and SCILBAUD specify the baud rate for the SCI.
 *
 *------------------------------------------------------------------------------------------------------------------*/
// 15-8 RESERVED
// 7-0  BAUD
// SCI 16-bit baud selection Registers SCILBAUD (MSbyte)


//  SCIHBAUD Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * The values in SCIHBAUD and SCILBAUD specify the baud rate for the SCI.
 *
 *------------------------------------------------------------------------------------------------------------------*/
// 15-8 RESERVED
// 7-0  BAUD
// SCI 16-bit baud selection Registers SCIHBAUD (MSbyte)

//  SCITXBUF Register
/*----------------------------------------------------------------------------------------------------------------
 *
 * Data bits to be transmitted are written to SCITXBUF. These bits must be right justified because the leftmost bits
 * are ignored for characters less than eight bits long. The transfer of data from this register to the TXSHF
 * transmitter shift register sets the TXRDY flag (SCICTL2.7), indicating that SCITXBUF is ready to receive another
 * set of data. If bit TXINTENA (SCICTL2.0) is set, this data transfer also causes an interrupt.
 *
 *------------------------------------------------------------------------------------------------------------------*/
// 15-8 RESERVED
// 7-0  TXDT
//


#endif /* COMMON_INCLUDE_F28004X_SCI_DEFINES_H_ */
