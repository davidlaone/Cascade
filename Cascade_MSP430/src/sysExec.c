/** 
 * @file sysExec.c
 * \n Source File
 * \n Cascade MSP430 Firmware
 * 
 * \brief main system exec that runs the top level loop.
 */

#include "cascade.h"

/***************************
 * Module Data Definitions
 **************************/

/**
 * \def START_UP_MSG_TX_DELAY_IN_SECONDS
 * \brief Two messages are transmitted after the system starts. 
 *        This definition specifies the delay in seconds after
 *        startup until the first startup message is sent to the
 *        modem for transmit. After the first startup message
 *        has completed transmitting, the same delay is used to
 *        determine when to transmit the second startup message.
 */
#define START_UP_MSG_TX_DELAY_IN_SECONDS ((uint8_t)30)

/**
 * \def SEND_DEBUG_INFO_TO_UART
 * \brief If set to 1, debug data is sent to the UART every 
 *        iteration of the main loop (i.e. every one second).
 */
#define SEND_DEBUG_INFO_TO_UART 0

/**
 * \def REBOOT_DELAY_IN_SECONDS
 * \brief This define is used in conjunction with the OTA reset
 *        device message.  It specifies how long to wait after
 *        the message was received to perform the MSP430 reset.
 */
#define REBOOT_DELAY_IN_SECONDS ((uint8_t)10*TIME_SCALER)

/**
 * \typedef sysExecData_t
 * \brief Container to hold data for the sysExec module.
 */
typedef struct sysExecData_s {
    bool startUpMsg1WasSent;         /**< Flag specifying if first startup msg was sent */
    bool startUpMsg2WasSent;         /**< Flag specifying if second startup msg was sent */
    int8_t secondsTillStartUpMsgTx;  /**< Seconds until startup messages are transmitted */
    uint16_t secondsTillReboot;      /**< How long to wait to perform a MSP430 reset */
    uint8_t rebootKeys[4];           /**< The received keys with the reset OTA message */
    bool rebootActive;               /**< Specify if a reboot OTA message was received */
} sysExecData_t;

/****************************
 * Module Data Declarations
 ***************************/

// static
sysExecData_t sysExecData;

/*************************
 * Module Prototypes
 ************************/

static void sleep(void);
static startUpMessageCheck(void);
static void sendStartUpMsg1(void);
static void sendStartUpMsg2(void);

/***************************
 * Module Public Functions
 **************************/

/**
* \brief This is the main infinite software loop for the MSP430 
*        firmware.  After calling the initialization routines,
*        it drops into an infinite while loop.  In the while
*        loop, it calls the exec routines of the different
*        modules and then goes into a low power mode (i.e.
*        sleep) waiting for the timer interrupt to wake it up.
*        The exec loop logic is setup to run exactly once every
*        one second.
* 
* \ingroup EXEC_ROUTINE
*/
void sysExec_exec(void) {

    WATCHDOG_TICKLE();

    // Clear the module data structure
    memset(&sysExecData, 0, sizeof(sysExecData_t));

    // Set how long to wait until first startup message should be transmitted
    sysExecData.secondsTillStartUpMsgTx = START_UP_MSG_TX_DELAY_IN_SECONDS;

    // Verify or initialize the flash parameter data base
    flashParam_init();

    // Start the timer interrupt
    timerA1_init();

    // Initialize the date for Jan 1, 2015
    // h, m, s, am/pm (must be in BCD)
    setTime(0x00, 0x00, 0x00, 0x00);
    // y, m, d
    setDate(2016, 1, 1);

    // Call the module init routines
    modemPower_init();
    modemCmd_init();
    modemMgr_init();
    dataMsgSm_init();
    dataMsgMgr_init();
    otaMsgMgr_init();
    dbgMsgMgr_init();
    storageMgr_init();
    flowSensor_init();

    WATCHDOG_TICKLE();

    // Enable the global interrupt
    enableGlobalInterrupt();

    // Start the infinite exec loop
    while (1) {

        WATCHDOG_TICKLE();

        // For timing debug
        P2_4_SET();

        // Water sensing and storage
        flowSensor_exec();

        // Handle all the water data statistics storage
        storageMgr_exec();

        // Communication
        modemCmd_exec();     /* perform Low-level modem interface processing */
        dataMsgMgr_exec();   /* perform High-level send data message */
        otaMsgMgr_exec();    /* perform High-level OTA Message Processing */
        modemMgr_exec();     /* perform Low-level message processing */
        modemCmd_exec();     /* perform Low-level modem interface processing (again) */
        modemPower_exec();    /* Handle powering on and off the modem */

#if (SEND_DEBUG_INFO_TO_UART==1)
        // Only send debug data if the modem is not in use.
        if (!modemMgr_isAllocated()) {
            sysExec_sendDebugDataToUart();
        }
#endif

        // For timing debug
        P2_4_CLEAR();

        // Sleep up to the one second mark.
        sleep();

        // If an OTA reset device message has been received, then rebootActive
        // will be true, waiting for the delay time to expire to perform
        // the MSP430 reboot.
        if (sysExecData.rebootActive) {
            if (sysExecData.secondsTillReboot > 0) {
                sysExecData.secondsTillReboot--;
            }
            if (sysExecData.secondsTillReboot == 0) {
                sysExec_doReboot();
            }
        }

        // Two messages are transmitted after the system starts.
        if (!sysExecData.startUpMsg1WasSent || !sysExecData.startUpMsg2WasSent) {
            startUpMessageCheck();
        }
    }
}
/***************************
 * Module Private Functions
 **************************/

/**
 * \brief There are two startup messages that must be sent. 
 *        Check the flags and delays to determine if the first
 *        one should be sent. Once the first startup message is
 *        sent, start the countdown to send the next one - but only
 *        after the first one has completed transmitting.
 *
 * \note The application record is updated after the first
 *       startup message completes. The record is written by the
 *       Application and used by the bootloader to help identify
 *       that the Application started successfully. The
 *       application record lives in one of the flash info
 *       sections.
 * 
 */
static startUpMessageCheck(void) {
    if (!sysExecData.startUpMsg1WasSent) {
        if (sysExecData.secondsTillStartUpMsgTx > 0) {
            sysExecData.secondsTillStartUpMsgTx--;
        }
        if (sysExecData.secondsTillStartUpMsgTx == 0) {
            sendStartUpMsg1();
            sysExecData.startUpMsg1WasSent = true;
            sysExecData.secondsTillStartUpMsgTx = START_UP_MSG_TX_DELAY_IN_SECONDS;
        }
    } else if (!sysExecData.startUpMsg2WasSent && !dataMsgMgr_isSendMsgActive()) {
        if (sysExecData.secondsTillStartUpMsgTx > 0) {
            sysExecData.secondsTillStartUpMsgTx--;
        }
        if (sysExecData.secondsTillStartUpMsgTx == 0) {
            // Update the Application Record after sending the first FA packet.
            // The record is written by the Application and used by the
            // bootloader to help identify that the Application started successfully.
            // We wait until after the first FA packet is sent to write the
            // Application Record in an attempt to help verify that the
            // application code does not have a catastrophic issue.
            // If the bootloader does not see an Application
            // Record when it boots, then it will go into recovery mode.
            //
            // Don't write a new Application Record if one already exists.  It is
            // erased by the bootloader after a new firmware upgrade has been
            // performed before jumping to the new Application code.
            // The Application Record is located in the flash INFO C section.
            if (!appRecord_checkForValidAppRecord()) {
                // If the record is not found, write one.
                appRecord_initAppRecord();
            }
            sendStartUpMsg2();
            sysExecData.startUpMsg2WasSent = true;
        }
    }
}

/**
* \brief Initiate sending the first startup message which is a 
*        Final Assembly Message Type
*/
static void sendStartUpMsg1(void) {
    // Prepare the final assembly message
    // Get the shared buffer (we borrow the ota buffer)
    uint8_t *payloadP = modemMgr_getSharedBuffer();
    // Fill in the buffer with the standard message header
    uint8_t payloadSize = storageMgr_prepareMsgHeader(payloadP, MSG_TYPE_FINAL_ASSEMBLY);
    // Initiate sending the final assembly message
    dataMsgMgr_sendDataMsg(MSG_TYPE_FINAL_ASSEMBLY, payloadP, payloadSize);
}

/**
* \brief Initiate sending the second startup message which is a 
*        Monthly Check-In Message Type
*/
static void sendStartUpMsg2(void) {
    // Prepare the monthly check-in message
    // Get the shared buffer (we borrow the ota buffer)
    uint8_t *payloadP = modemMgr_getSharedBuffer();
    // Fill in the buffer with the standard message header
    uint8_t payloadSize = storageMgr_prepareMsgHeader(payloadP, MSG_TYPE_CHECKIN);
    // Initiate sending the monthly check-in message
    dataMsgMgr_sendDataMsg(MSG_TYPE_CHECKIN, payloadP, payloadSize);
}

/**
* \brief Sleep for up to 1 second.  Because the maximum watchdog 
*        timeout is one second, the sleep is broken into half
*        second segments.  The system timer interrupt is
*        programmed to interrupt every half second, which will
*        wake the processor from sleep.
*/
static void sleep(void) {
    // P2OUT |= DEBUG0;
    WATCHDOG_TICKLE();
    // sleep, wake on Timer1A interrupt
    __bis_SR_register(LPM3_bits);
    _nop();
    WATCHDOG_TICKLE();
    // P2OUT &= ~DEBUG0;
    // sleep, wake on Timer1A interrupt
    __bis_SR_register(LPM3_bits);
    _nop();
    WATCHDOG_TICKLE();
}

/**
* \brief Support utility for the OTA message that resets the 
*        unit.  Checks to make sure the message keys are
*        correct, and if so, then starts a countdown counter for
*        rebooting the unit.
* 
* @return bool  Returns true if keys were correct.
*/
bool sysExec_startRebootCountdown(uint8_t *keysP) {
    bool status = false;
    if (keysP[0] == REBOOT_KEY1 &&
        keysP[1] == REBOOT_KEY2 &&
        keysP[2] == REBOOT_KEY3 &&
        keysP[3] == REBOOT_KEY4) {
        memcpy(sysExecData.rebootKeys, keysP, 4);
        sysExecData.secondsTillReboot = REBOOT_DELAY_IN_SECONDS;
        sysExecData.rebootActive = true;
        status = true;
    }
    return status;
}

/**
* \brief Support routine that is called to perform a system 
*        reboot.  Called as a result of receiving the OTA reset
*        unit message.
*/
void sysExec_doReboot(void) {
    if (sysExecData.rebootActive) {
        if (sysExecData.rebootKeys[0] == REBOOT_KEY1 &&
            sysExecData.rebootKeys[1] == REBOOT_KEY2 &&
            sysExecData.rebootKeys[2] == REBOOT_KEY3 &&
            sysExecData.rebootKeys[3] == REBOOT_KEY4) {
            // Disable the global interrupt
            disableGlobalInterrupt();
            // Modem should already be off, but just for safety, turn it off
            modemPower_powerDownModem();
            while (1) {
                // Force watchdog reset
                WDTCTL = 0xDEAD;
                while (1);
            }
        } else {
            sysExecData.rebootActive = false;
        }
    }
}

#if (SEND_DEBUG_INFO_TO_UART==1)
/**
* \brief Send debug information to the uart.  
*/
void sysExec_sendDebugDataToUart(void) {
    // Get the shared buffer (we borrow the ota buffer)
    uint8_t *payloadP = modemMgr_getSharedBuffer();
    uint8_t payloadSize = storageMgr_prepareMsgHeader(payloadP, MSG_TYPE_DEBUG_TIME_INFO);
    dbgMsgMgr_sendDebugMsg(MSG_TYPE_DEBUG_TIME_INFO, payloadP, payloadSize);
    _delay_cycles(10000);
    storageMgr_sendDebugDataToUart();
    _delay_cycles(10000);
    flowSensor_sendDebugDataToUart();
    _delay_cycles(10000);
}
#endif

/***************************
 * Module Private Functions
 **************************/

