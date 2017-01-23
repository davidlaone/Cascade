/** 
 * @file hal.c
 * \n Source File
 * \n Cascade MSP430 Bootloader Firmware
 * 
 * \brief Hal support routines
 */

#include "cascade.h"

/**
 * \note From the data sheet: Unused I/O pins should be 
 * configured as I/O  function, output direction, and left 
 * unconnected on the PC board, to prevent a floating input and 
 * reduce power consumption. The value of the PxOUT bit is 
 * irrelevant, since the pin is unconnected. Alternatively, the 
 * integrated pullup/pulldown resistor can be enabled by setting 
 * the PxREN bit of the unused pin to prevent the floating 
 * input. 
 *
 * \note Dir control bits: 
 * \li Bit = 0: The port pin is switched to input direction
 * \li Bit = 1: The port pin is switched to output direction
 *
 * \note After reset, all port pins are set as an input by
 *       default
 */

// Port 1 pins
// ======================
// P1.0 = Output, LS_VCC for buffer driver
// P1.1 = unconnected
// P1.2 = unconnected
// P1.3 = unconnected
// P1.4 = Input, Modem Status (not used)
// P1.5 = unconnected
// P1.6 = Output, GSM_POWER
// P1.7 = Output, DCDC_EN
// -Input MASK   = 0x10
// -Output MASK  = 0xC1
// -Unused Mask  = 0x2E
// -Special Mask = 0x00;

// Port 2 pins
// ======================
// P2.0 = unconnected
// P2.1 = unconnected
// P2.2 = Input, Flow Pulse Input (not used)
// P2.3 = unconnected
// P2.4 = unconnected
// P2.5 = unconnected
// P2.6 = Special, XTAL XIN
// P2.7 = Special, XTAL XOUT
// -Input MASK   = 0x04
// -Output MASK  = 0x00
// -Unused Mask  = 0x3B
// -Special Mask = 0xC0;

// Port 3 pins
// ======================
// P3.0 = unconnected
// P3.1 = Special, SDA, Temp & Humidity Chip
// P3.2 = Special, SCL, Temp & Humidity Chip
// P3.3 = Output, Temp & Humidity Chip power, I2C Pullup.
// P3.4 = Special, UART TX
// P3.5 = Special, UART RX
// P3.6 = unconnected
// P3.7 = unconnected
// -Input MASK   = 0x00
// -Output MASK  = 0x08
// -Unused Mask  = 0xC1
// -Special Mask = 0x36;

// Port 4 pins
// ======================
// P4.0 = Special, Flow Pulse Input for Timer
// P4.1 = unconnected
// P4.2 = unconnected
// P4.3 = unconnected
// P4.4 = unconnected
// P4.5 = unconnected
// P4.6 = unconnected
// P4.7 = unconnected
// -Input MASK   = 0x00
// -Output MASK  = 0x00
// -Unused Mask  = 0x7E
// -Special Mask = 0x01;

/**
* \brief One time init of all gpio port pin related items after 
*        boot up.
* \ingroup PUBLIC_API
*/
void pin_init(void) {

    // Port 1 IO Setup
    P1DIR |= (GSM_EN | GSM_DCDC | LS_VCC);
    // Set IO output to low
    P1OUT &= ~(GSM_EN | GSM_DCDC | LS_VCC);

    // Port 2 IO Setup
    P2DIR |= BIT7; // Pins 6/7 are XIN/XOUT
    P2SEL |= BIT6 + BIT7;

    // Port 3 IO Setup
    P3DIR |= TEMP_PWR;
    // Set IO output to low
    P3OUT &= ~(TEMP_PWR);

    // Port 4 IO Setup
    // No Output IO

    // Port 1 unconnected pins - set as output
    P1DIR |= 0x2E;
    // Port 2 unconnected pins - set as output
    P2DIR |= 0x3B;
    // Port 3 unconnected pins - set as output
    P3DIR |= 0xC1;
    // Port 4 unconnected pins - set as output
    P4DIR |= 0x7E;

#ifdef DEBUG_IO
    // Port 2 Debug IO
    P2REN |= (DEBUG_P2_3 | DEBUG_P2_4); // P2.3,P2.4 are debug output
    P2DIR |= (DEBUG_P2_3 | DEBUG_P2_4); // P2.3,P2.4 are debug output

    // Output A clock on P2.0 for Debug
    P2DIR |= 0x01; // P2.0 output
    P2SEL |= 0x01; // P2.0 ACLK output
#endif

}

/**
* \brief One time init of the Uart subsystem after boot up.
* \ingroup PUBLIC_API
*/
void uart_init(void) {
    // Setup I/O for UART
    P3SEL  |= RXD + TXD; // P3.5 = RXD, P3.4=TXD

    //  ACLK source for UART
    UCA0CTL1 |= UCSSEL_1; // ACLK
    UCA0BR0 = 0x03; // 32 kHz 9600
    UCA0BR1 = 0x00; // 32 kHz 9600
    UCA0MCTL = UCBRS0 + UCBRS1; // Modulation UCBRSx = 3
    UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
}

/**
* \brief One time init of all clock related subsystems after 
*        boot up.
* \ingroup PUBLIC_API
*/
void clock_init(void) {
    DCOCTL = 0;               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;    // Set DCO
    DCOCTL = CALDCO_1MHZ;

    BCSCTL1 |= DIVA_0;        // ACLK/1 [ACLK/(0:1,1:2,2:4,3:8)]
    BCSCTL2 = 0;              // SMCLK [SMCLK/(0:1,1:2,2:4,3:8)]
    BCSCTL3 |= LFXT1S_0;      // LFXT1S0 32768-Hz crystal on LFXT1
}

