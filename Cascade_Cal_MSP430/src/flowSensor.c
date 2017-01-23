/** 
 * @file flowSensor.c
 * \n Source File
 * \n Cascade MSP430 Firmware
 * 
 * \brief Routines to support flow sensor pulse count and 
 *        algorithm.
 */

/********************************************************************************
    Flowsensor pulse rate to liters per second calculation.
    Historically, the original function provided by the flow sensor
    manufacturer was as follows:
 
    Original From Manufacturer:
    Liters per minute = (pulse_count (HZ) + 6) / 8.1

    Convert to Milliliters per Second
    ML per second = (((pulse_count (HZ) + 6) / 8.1) * 1000) / 60
 
    Combine constants
    ML per second = (pulse_count(HZ) + 6) * 2.0576
 
    Note that this is a linear equation (ax+b), a = slope, b = offset.
    For the current design, the default constants are not used.  Rather, we
    calibrate a flowsensor to obtain  the slope and offset. These are stored in
    the flash parameter section during calibration at manufacturing time.
********************************************************************************/

#include "cascade.h"

/***************************
 * Module Data Definitions
 **************************/

/**
 * \typedef Define module Data structure
 */
typedef struct flowSensorData_s {
    uint16_t pulseCount;      /**< Resultant pulse count after one second (milliliters) */
    uint16_t flowRateInML;    /**< Calculated flow rate performed each second */
    uint16_t maxFlowRateInML; /**< Track the max flow rate observed over one second */
    uint16_t isrPulseCount;   /**< If gpio isr is used, incremented by one count every ISR */
    uint32_t secondsOfFlow;   /**< Track which seconds had pulse counts */
    uint32_t totalPulseCount; /**< Track total pulse counts */
} flowSensorData_t;

/****************************
 * Module Data Declarations
 ***************************/

/**
* \var flowSensorData
* \brief Declare module data structure.
*/
flowSensorData_t flowSensorData;

/*************************
 * Module Prototypes
 ************************/

#ifdef USE_TIMER_FOR_PULSE_COUNTING
static uint16_t timerB0_restart(void);
#else
static void flow_sensor_pin_init(void);
#endif
static void flowSensor_calculateVolume(uint16_t pulseCount);

/***************************
 * Module Public Functions
 **************************/

/**
* \brief Call once as system startup to initialize the module. 
* \ingroup PUBLIC_API
*/
void flowSensor_init(void) {
    memset(&flowSensorData, 0, sizeof(flowSensorData_t));
#ifdef USE_TIMER_FOR_PULSE_COUNTING
    // TimerB is used to count the input flow pulses.
    timerB0_restart();
#else
    flow_sensor_pin_init();
#endif
}

/**
* \brief This is the flow sensor exec. It is called from the 
*        main processing loop.
* \ingroup EXEC_ROUTINE
*/
void flowSensor_exec(void) {

#ifdef USE_TIMER_FOR_PULSE_COUNTING
    // Read the timerB counter register value and
    // then restart the timerB counter.
    flowSensorData.pulseCount = timerB0_restart();
#else
    // Disable Interrupt
    P2IE  &=  ~BIT2;

    // Read the total pulses that the ISR incremented in the last second
    flowSensorData.pulseCount = flowSensorData.isrPulseCount;
    // Restart ISR counter
    flowSensorData.isrPulseCount = 0;

    // Enable Interrupt
    P2IE  |=   BIT2;
#endif

    if (flowSensorData.pulseCount) {
        // track if the last second had any flow (i.e. pulses)
        flowSensorData.secondsOfFlow++;
        // track total pulse count
        flowSensorData.totalPulseCount += flowSensorData.pulseCount;
        // Perform the conversion from pulses per second to Milliliters per second
        flowSensor_calculateVolume(flowSensorData.pulseCount);
    } else {
        flowSensorData.flowRateInML = 0;
    }
}

/**
* \brief Return the last pulse count measured (previous second 
*        of data).
* \ingroup PUBLIC_API
* 
* @return uint16_t  Pulse Count
*/
uint16_t flowSensor_getLastPulseCount(void) {
    return flowSensorData.pulseCount;
}

/**
* \brief Return the last flow rate measured
* \ingroup PUBLIC_API
* 
* @return uint16_t  Flow rate in milliliters per second
*/
uint16_t flowSensor_getLastMeasFlowRateInML(void) {
    return flowSensorData.flowRateInML;
}

/**
* \brief Return the maximum flow rate per second (in 
*        milliliters) that has been calculated.
* \note Calling this function will result in the internal 
*       maxFlowRateInML statistic to be zero'd.
* \ingroup PUBLIC_API
* 
* @return uint16_t The max flow rate per second calculated since
*         the last time this function was called.
*/
uint16_t flowSensor_getMaxFlowRateInML(void) {
    uint16_t temp16 = flowSensorData.maxFlowRateInML;
    flowSensorData.maxFlowRateInML = 0;
    return temp16;
}

/**
* \brief Return the total seconds-of-flow statistic. 
* \note Calling this function will result in the internal 
*       seconds-of-flow statistic to be zero'd.
* \ingroup PUBLIC_API
* 
* @return uint32_t The total number of seconds which flow pulses
*         occurred since the last time this function was called.
*/
uint32_t flowSensor_getSecondsOfFlow(void) {
    uint32_t temp32 = flowSensorData.secondsOfFlow;
    flowSensorData.secondsOfFlow = 0;
    return temp32;
}

/**
* \brief Return the total pulse count statistic. 
* \note Calling this function will result in the internal total 
*       pulse count statistic to be zero'd.
* \ingroup PUBLIC_API
* 
* @return uint32_t The total number of flow pulses counted since
*         the last time this function was called.
*/
uint32_t flowSensor_getTotalPulseCount(void) {
    uint32_t temp32 = flowSensorData.totalPulseCount;
    flowSensorData.totalPulseCount = 0;
    return temp32;
}

/**
* \brief Send flow sensor debug information to the uart.  Dumps 
*        the complete flowData data structure.
* \ingroup PUBLIC_API
* 
*/
void flowSensor_sendDebugDataToUart(void) {
    // Get the shared buffer (we borrow the ota buffer)
    uint8_t *payloadP = modemMgr_getSharedBuffer();
    // Add structure size plus two for start byte and payload msg ID
    uint8_t payloadSize = sizeof(flowSensorData_t) + 2;
    // Add payload start byte
    payloadP[0] = 0x1;
    // Add payload msg ID
    payloadP[1] = MSG_TYPE_DEBUG_FLOW_SENSOR_DATA;
    // Add debug data
    memcpy(&payloadP[2], &flowSensorData, sizeof(flowSensorData_t));
    // Output debug information
    dbgMsgMgr_sendDebugMsg(MSG_TYPE_DEBUG_FLOW_SENSOR_DATA, payloadP, payloadSize);
}

/*************************
 * Module Private Functions
 ************************/

/**
* \brief Perform a water volume calculation based on the passed 
*        in pulse count.
* 
* @param pulseCount Pulse count measurement over one second as 
*                   performed by the flowSensor module.
*/
static void flowSensor_calculateVolume(uint16_t pulseCount) {
    float slope;
    float offset;
    float flowRateInML;
    uint16_t slopeX1000;
    int16_t offsetX1000;
    // Read the water volume constants from the flash parameter data base.
    // The constants are stored as integers x 1000.
    flashParam_getFlowConstants(&slopeX1000, &offsetX1000);
    // Convert to float with scaling removed
    slope = slopeX1000 / 1000.0;
    offset = offsetX1000 / 1000.0;
    // Perform calculation
    flowRateInML = (pulseCount * slope) + offset;
    flowSensorData.flowRateInML = flowRateInML;
    // Track the largest flow rate observed
    if (flowSensorData.flowRateInML > flowSensorData.maxFlowRateInML) {
        flowSensorData.maxFlowRateInML = flowSensorData.flowRateInML;
    }
}

#ifdef USE_TIMER_FOR_PULSE_COUNTING
/**
* \brief Initialize timerB0 to count flow pulses. The pulses are
*        used to clock the timer via the external clock input on
*        pin 24, P4.7 of the 2955.
* 
* @return uint16_t Returns the 16 bit value of the timer count 
*         register.
*/
static uint16_t timerB0_restart(void) {
    uint16_t count;
    // Stop the timer
    TB0CTL = MC_0;
    // Read the counter register
    count = TB0R;
    // Re - Initialize for continuous-counter, TB0CLK (external on P4.7)
    TB0CTL = TBSSEL_0 + MC_2 + TACLR;

    return count;
}
#else

// Interrupt support for flow pulse counting
// Uses port 2, bit 2 as IO input.

/**
* \brief Interrupt routine for flow pulse counting.
*/
#ifndef FOR_USE_WITH_BOOTLOADER
#pragma vector=PORT2_VECTOR
#endif
__interrupt void flowSensorISR(void) {
    P2_3_SET();
    // Clear interrupt
    P2IFG &= ~BIT2;
    // Increment debug counter
    flowSensorData.isrPulseCount++;
    P2_3_CLEAR();
}

/**
* \brief Setup port 2, bit 2 as an IO input interrupt.
*/
static void flow_sensor_pin_init(void) {
    // Set interrupt on Port 2 BIT2, rising edge
    P2DIR &= ~BIT2;   // Set as input
    P2IE  |=  BIT2;   // Interrupt enable
    P2IES |=  BIT2;   // Falling Edge
    P2IFG &= ~BIT2;   // Clear interrupt
}
#endif

