/** 
 * @file hal.c
 * \n Source File
 * \n Cascade MSP430 Firmware
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
// P2.0 = Input, JIG detect (CAL ONLY)
// P2.1 = Output, JIG Yellow LED (CAL ONLY)
// P2.2 = Input, Flow Pulse Input (not used)
// P2.3 = Output, JIG Red LED (CAL ONLY)
// P2.4 = Output, JIG Green LED (CAL ONLY)
// P2.5 = unconnected
// P2.6 = Special, XTAL XIN
// P2.7 = Special, XTAL XOUT
// -Input MASK   = 0x05
// -Output MASK  = 0x1A
// -Unused Mask  = 0x20
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
void hal_pinInit(void) {

    // Port 1 IO Setup
    P1DIR |= (GSM_EN | GSM_DCDC | LS_VCC);
    // Set IO output to low
    P1OUT &= ~(GSM_EN | GSM_DCDC | LS_VCC);

    // Port 2 IO Setup
    // Low Frequency Crystal Setup (Pins 6/7 are XIN/XOUT)
    P2DIR |= X_OUT_P2_7;
    P2SEL |= (X_IN_P2_6 | X_OUT_P2_7);

    // Port 3 IO Setup
    P3DIR |= NFC_CHIP_POWER;
    // Set IO output to low
    P3OUT &= ~(NFC_CHIP_POWER);

    // Port 4 IO Setup
    // Setup P4.7 I/O as TB0CLK input (for TimerB clock input from pulses)
    P4DIR  &= ~0x80;  // Set as input
    P4SEL  |=  0x80;  // Set P4SEL.7 = TB0CLK input
    P4SEL2 &= ~0x80;  // Clear P4SEL2.7

    // Port 1 unconnected pins - set as output
    P1DIR |= 0x2E;
    // Port 2 unconnected pins - set as output
    P2DIR |= 0x20;
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

    // -- Test JIG I/O --

    // Test JIG Detect input on P2_0
    P2DIR &= ~JIG_DETECT_P2_0;  // Set to input
    P2REN |=  JIG_DETECT_P2_0;  // Set Pull Up/Pull Down
    P2OUT &= ~JIG_DETECT_P2_0;  // Enable pull down

    // Test Jig LEDs Set to Outputs
    P2DIR |= (JIG_YELLOW_LED_P2_1 |  JIG_RED_LED_P2_3 | JIG_GREEN_LED_P2_4);  

    // Test JIG Init LED GPIO
    hal_yellowLedOff();
    hal_redLedOff();
    hal_greenLedOff();
}

/**
* \brief One time init of the Uart subsystem after boot up.
* \ingroup PUBLIC_API
*/
void hal_uartPinInit(void) {
    // Setup I/O for UART
    P3SEL  |= UART_RXD + UART_TXD; // P3.5 = RXD, P3.4=TXD

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
void hal_sysClockInit(void) {
    DCOCTL = 0;               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;    // Set DCO
    DCOCTL = CALDCO_1MHZ;

    BCSCTL1 |= DIVA_0;        // ACLK/1 [ACLK/(0:1,1:2,2:4,3:8)]
    BCSCTL2 = 0;              // SMCLK [SMCLK/(0:1,1:2,2:4,3:8)]
    BCSCTL3 |= LFXT1S_0;      // LFXT1S0 32768-Hz crystal on LFXT1
}

/****************** 
1) Check GPIO (P2.0; only continue with this process if set high)
2) Sleep (and wait for water event to complete)
3) Set yellow LED high (PT2.1)
4) Power on modem
5) Issue SEND_TEST command
6) Issue GET_STATUS command
7a) If state >= 0x80, set red LED high (P2.3) & go to step 10
7b) If state == 0x04, set green LED high (2.4) & continue
7c) else sleep for a second & go to step 6
8) Issue INFO command (to get IMEI)
9) Program NFC tag
10) Issue POWER_OFF command
11) while(1); 
 
JIG Detect GPIO: P2.0
Yellow LED:      P2.1
Red LED:         P2.3
Green LED:       P2.4

*****************/

/**
* @brief Determine if board is in the Test JIG
* 
* @return bool Returns true if test jig detected.  False otherwise. 
*/
bool hal_testJigDetected(void) {
    bool jigDetected = false;
    if (P2IN & JIG_DETECT_P2_0) {
        jigDetected = true;
    }
    return jigDetected;
}

/**
* @brief Test Jig LED CONTROL
*/
void hal_yellowLedOn(void) {
    P2OUT |= JIG_YELLOW_LED_P2_1;
}
/**
* @brief Test Jig LED CONTROL
*/
void hal_yellowLedOff(void) {
    P2OUT &= ~JIG_YELLOW_LED_P2_1;
}
/**
* @brief Test Jig LED CONTROL
*/
void hal_redLedOn(void) {
    P2OUT |= JIG_RED_LED_P2_3;
}
/**
* @brief Test Jig LED CONTROL
*/
void hal_redLedOff(void) {
    P2OUT &= ~JIG_RED_LED_P2_3;
}
/**
* @brief Test Jig LED CONTROL
*/
void hal_greenLedOn(void) {
    P2OUT |= JIG_GREEN_LED_P2_4;
}
/**
* @brief Test Jig LED CONTROL
*/
void hal_greenLedOff(void) {
    P2OUT &= ~JIG_GREEN_LED_P2_4;
}
