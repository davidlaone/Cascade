/** 
 * @file time.c
 * \n Source File
 * \n Cascade MSP430 Firmware
 * 
 * \brief MSP430 sleep control and support routines
 */

/*
*  Note - the timer naming convention is very confusing on the MSP430
*
*  For the MSP430G2953, there are three timers on the system:  TimerA0, TimerA1
*    & TimerB0.  Each timer has three capture & control channels: 0,1 & 2.
* 
*  Timer0_A3 and Timer1_A3 are 16-bit timers/counters with three capture/compare registers.
*  Timer0_B3 is a 16-bit timer/counter with three capture/compare registers.
* 
*  Naming Usage:
*  TimerA0_0, Timer A0, capture control channel 0
*  TimerA0_1, Timer A0, capture control channel 1
*  TimerA0_2, Timer A0, capture control channel 2
*  TimerA1_0, Timer A1, capture control channel 0
*  TimerA1_1, Timer A1, capture control channel 1
*  TimerA1_2, Timer A1, capture control channel 2
*  TimerB0_0, Timer B1, capture control channel 0
*  TimerB0_1, Timer B1, capture control channel 1
*  TimerB0_2, Timer B1, capture control channel 2
*
*  Timer allocation/usage for Cascade APPLICATION:
*    A1, Capture control channel 0 used for system tick (with ISR, vector = 13 @ 0FFE2h)
*
*  Timer allocation/usage for Cascade BOOT:
*    A0, Capture control channel 0 used for system tick (with ISR, vector = 9 @ 0FFF2h)
* 
* The interrupt vector naming corresponds as follows:
*  TIMER0_A0_VECTOR,".int09",0xFFF2 Timer0_A CC0 
*  TIMER0_A1_VECTOR,".int08",0xFFF0 Timer0_A CC1, TA0 
*  TIMERB0_VECTOR,".int13",0xFFFA Timer B CC0 
*  TIMERB1_VECTOR,".int12", 0xFFF8 Timer B CC1-6, TB 
*  TIMER1_A0_VECTOR,".int01",0xFFE2 Timer1_A CC0 
*  TIMER1_A1_VECTOR,".int00",0xFFE0 Timer1_A CC1-4, TA1 
* =============================================================================== 
* From the MSP430 Documentation:
* 
* Up mode operation 
* The timer repeatedly counts up to the value of compare register TACCR0, which defines the period.
* The number of timer counts in the period is TACCR0+1. When the timer value equals TACCR0 the timer
* restarts counting from zero. If up mode is selected when the timer value is greater than TACCR0, the
* timer immediately restarts counting from zero.
* =============================================================================== 
* Timer Registers for Timer A0 and A1  (x = 0 or 1 accordingly)
*  TAxCTL - reg to setup timer counting type, clock, etc - CONTROLS ALL OF TIMER A1
*  Capture/Control channel 0
*  TAxCCR0 - specify the compare count for channel 0
*  TAxCCTL0 - setup interrupt for channel 0 
*  Capture/Control channel 1
*  TAxCCR1 - specify the compare count for channel 1
*  TAxCCTL1 - setup interrupt for channel 1
*/

#include "cascade.h"
#include "RTC_Calendar.h"

#pragma NOINIT(tp)

/**
* \var tp
* \brief Internal static structure to hold the time in binary or 
*        bcd format.  Pointer to this structure is returned by
*        the getBinTime and getBcdTime functions.
*/
static timePacket_t tp;

/**
* \var half_seconds_since_boot
* \brief Declare half seconds since boot counter.  Incremented by timer 
*        interrupt routine.
*/
static volatile uint32_t half_seconds_since_boot = 0;

/**
* \brief Initialize and start timerA1 for the half second system
*        tick.  Uses Timer A1, capture/control channel 0, vector
*        13, 0xFFFA.  The counter increments at a rate of 32768 HZ.
* \ingroup PUBLIC_API
*/
void timerA1_init(void) {
    // Set up Timer1 to wake every half second, counting to 16384 (0x4000) aclk cycles
    // Documentation states match will occur at (count+1)
    TA1CCR0 = 0x4000 - 1;
    TA1CTL = TASSEL_1 + MC_1 + TACLR;
    TA1CCTL0 &= ~CCIFG;
    TA1CCTL0 |= CCIE;
}

/**
* \brief Retrieve the seconds since boot system value.
* \ingroup PUBLIC_API
* 
* @return uint32_t 32 bit values representing seconds since the 
*         system booted.
*/
uint32_t getSecondsSinceBoot(void) {
    return (half_seconds_since_boot >> 1);
}

/**
* \brief Timer ISR. Produces the 2HZ timer tick interrupt. 
*        Uses Timer A1, capture/control channel 0, vector 13,
*        0xFFFA.
* \ingroup ISR
*/
#ifndef FOR_USE_WITH_BOOTLOADER
#pragma vector=TIMER1_A0_VECTOR
#endif
__interrupt void ISR_Timer1_A0(void) {
    TA1CTL |= TACLR;
    // Increment the half seconds counter
    half_seconds_since_boot++;
    // Increment seconds every other interrupt
    if (half_seconds_since_boot & 1) {
        // Increment the TI calendar library
        incrementSeconds();
    }
    // Because we want to wake the processor, use the following
    // to set the power control bits on the MSP430.
    __bic_SR_register_on_exit(LPM3_bits);
}

#if 0
void calibrateLoopDelay (void){
    P1DIR |= BIT3;
    while (1) {
        P1OUT &= ~BIT3;
        _delay_cycles(1000);
        P1OUT |= BIT3;
        _delay_cycles(1000);
    }
}
#endif

/**
* \brief Utility function to get the time from the TI calender 
*        module and convert to binary format.
* \note Only tens is returned for year (i.e. 15 for 2015, 16 for
*       2016, etc.)
* 
* @return timePacket_t* Pointer to a time packet structure 
*           to fill in with the time.
*/
timePacket_t* getBinTime(void) {
    uint16_t mask;
    mask = getAndDisableSysTimerInterrupt();
    tp.second  = bcd_to_char(TI_second);
    tp.minute  = bcd_to_char(TI_minute);
    tp.hour24  = bcd_to_char(get24Hour());
    tp.day     = bcd_to_char(TI_day);
    tp.month   = bcd_to_char(TI_month) + 1; // correcting from RTC lib's 0-indexed month
    tp.year    = bcd_to_char((TI_year)&0xFF);
    restoreSysTimerInterrupt(mask);
    return &tp;
}

/**
* \brief Utility function to get the time from the TI calendar
*        module in bcd format.
*  
* \note Month is returned as BCD zero based.  Only tens is 
*       returned for year (i.e. 15 for 2015, 16 for 2016, etc.)
* 
* @return timePacket_t* Pointer to a time packet structure 
*           to fill in with the time.
*/
timePacket_t* getBcdTime(void) {
    uint16_t mask;
    mask = getAndDisableSysTimerInterrupt();
    tp.second  = TI_second;
    tp.minute  = TI_minute;
    tp.hour24  = get24Hour();
    tp.day     = TI_day;
    tp.month   = TI_month; // currently 0 based
    tp.year    = ((TI_year)&0xFF);
    restoreSysTimerInterrupt(mask);
    return &tp;
}

/**
 * \brief Utility function to convert a byte of bcd data to a 
 *        binary byte.  BCD data represents a value of 0-99,
 *        where each nibble of the input byte is one decimal
 *        digit.
 * 
 * @param bcdValue Decimal value, each nibble is a digit of 0-9.
 * 
 * @return uint8_t binary conversion of the BCD value.
 *  
 */
uint8_t bcd_to_char(uint8_t bcdValue) {
    uint8_t tenHex = (bcdValue >> 4) & 0x0f;
    uint8_t tens = (((tenHex << 2) + (tenHex)) << 1);
    uint8_t ones = bcdValue & 0x0f;
    return (tens + ones);
}

/**
* \brief Convert a binary byte to its bcd representation. 
* \note WARNING - this only function works for binary values 
*       less than 100 (0x64)!
* 
* @param binVal binary value 0-99
* 
* @return uint8_t A value whose decimal representation of the 
*         binVal number is "visually" shown in the returned hex
*         value.  For example, an input value of 0x10 returns a
*         value of 0x16.  An input value of 0x20 returns a value
*         of 0x32.
*/
uint8_t char_to_bcd(uint8_t binVal) {
    uint8_t upperNibble = 0;
    uint8_t lowerNibble = 0;
    lowerNibble = binVal % 10;
    binVal -= lowerNibble;
    upperNibble =  binVal / 10;
    return (upperNibble << 4) | lowerNibble;
}

