/** 
 * @file cascade.h
 * \n Source File
 * \n Cascade MSP430 bootloader Firmware
 * 
 * \brief MSP430 System wide header file for Cascade firmware. 
 *        Contains function prototypes, MACROS, and data
 *        definitions for all "C" modules in the Cascade
 *        bootloader firmware.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "msp430.h"
#include "msp430g2955.h"
#include "modemMsg.h"

/**
 * \def CASCADE_PRODUCT_ID
 * \brief Specify the cascade product ID number that is sent in 
 *        messages.
 */
#define CASCADE_PRODUCT_ID ((uint8_t)2)

/**
 * \def FW_VERSION_MAJOR
 * \brief Specify the cascade firmware major version number.
 */
#define FW_VERSION_MAJOR ((uint8_t)0x02)

/**
 * \def FW_VERSION_MINOR
 * \brief Specify the cascade firmware minor version number.
 */
#define FW_VERSION_MINOR ((uint8_t)0x03)

/*******************************************************************************
* System Tick Access
*******************************************************************************/
// Define the type that a system tick value is represented in
typedef uint32_t sys_tick_t;
// Just return the system tick variable value.
#define GET_SYSTEM_TICK() ((sys_tick_t)getSysTicksSinceBoot());
// Return the number of elapsed system ticks since x
#define GET_ELAPSED_SYS_TICKS(x) (((sys_tick_t)getSysTicksSinceBoot())-(sys_tick_t)(x))

/**
 * \def SYS_TICKS_PER_SECOND
 * \brief Identify how many sys ticks occur per second. The main 
 *        exec loop runs at sys tick rate.
 */
#define SYS_TICKS_PER_SECOND ((uint16_t)32)

/*******************************************************************************
* MISC Macros
*******************************************************************************/
/**
 * \def TIME_5_SECONDS 
 * \brief Macro for 5 seconds
 */
#define TIME_5_SECONDS ((uint8_t)5)
/**
 * \def TIME_10_SECONDS 
 * \brief Macro for 10 seconds
 */
#define TIME_10_SECONDS ((uint8_t)10)
/**
 * \def TIME_20_SECONDS 
 * \brief Macro for 20 seconds
 */
#define TIME_20_SECONDS ((uint8_t)20)
/**
 * \def TIME_30_SECONDS 
 * \brief Macro fo 30 seconds
 */
#define TIME_30_SECONDS ((uint8_t)30)
/**
 * \def TIME_60_SECONDS 
 * \brief Macro for 60 seconds
 */
#define TIME_60_SECONDS ((uint8_t)60)
/**
 * \def SECONDS_PER_MINUTE
 * \brief Macro to specify seconds per minute
 */
#define SECONDS_PER_MINUTE ((uint8_t)60)
/**
 * \def SECONDS_PER_HOUR
 * \brief Macro to specify seconds per hour
 */
#define SECONDS_PER_HOUR (SECONDS_PER_MINUTE*((uint16_t)60))
/**
 * \def TIME_ONE_HOUR
 * \brief Macro to specify one hour in terms of seconds
 */
#define TIME_ONE_HOUR SECONDS_PER_HOUR
/**
 * \def SEC_PER_MINUTE
 * \brief Macro to specify the number of seconds in one minute
 */
#define SEC_PER_MINUTE ((uint8_t)60)
/**
 * \def TIME_45_MINUTES
 * \brief Macro to specify the number of seconds in 45 minutes
 */
#define TIME_45_MINUTES ((uint16_t)(SEC_PER_MINUTE*(uint16_t)45))
/**
 * \def TIME_60_MINUTES
 * \brief Macro to specify the number of seconds in 60 minutes
 */
#define TIME_60_MINUTES ((uint16_t)(SEC_PER_MINUTE*(uint16_t)60))
/**
 * \def TIME_5_MINUTES
 * \brief Macro to specify the number of seconds in 5 minutes
 */
#define TIME_5_MINUTES ((uint16_t)(SEC_PER_MINUTE*(uint16_t)5))
/**
 * \def TIME_10_MINUTES
 * \brief Macro to specify the number of seconds in 10 minutes
 */
#define TIME_10_MINUTES ((uint16_t)(SEC_PER_MINUTE*(uint16_t)10))
/**
 * \def TIME_20_MINUTES
 * \brief Macro to specify the number of seconds in 10 minutes
 */
#define TIME_20_MINUTES ((uint16_t)(SEC_PER_MINUTE*(uint16_t)20))

/* Give names to various I/O Port Bits */
#define LS_VCC     BIT0
#define RXD        BIT5
#define TXD        BIT4
#define GSM_STATUS BIT4
#define GSM_EN     BIT6
#define GSM_DCDC   BIT7
#define TEMP_PWR   BIT3
#define DEBUG_P2_3 BIT3
#define DEBUG_P2_4 BIT4

// #define DEBUG_IO
#ifdef DEBUG_IO
#define P2_3_TOGGLE() (P2OUT ^= BIT3)
#define P2_3_SET() (P2OUT |= BIT3)
#define P2_3_CLEAR() (P2OUT &= ~BIT3)
#define P2_4_TOGGLE() (P2OUT ^= BIT4)
#define P2_4_SET() (P2OUT |= BIT4)
#define P2_4_CLEAR() (P2OUT &= ~BIT4)
#else
#define P2_3_TOGGLE()
#define P2_3_SET()
#define P2_3_CLEAR()
#define P2_4_TOGGLE()
#define P2_4_SET()
#define P2_4_CLEAR()
#endif

/*******************************************************************************
*  Centralized method for enabling and disabling MSP430 interrupts
*******************************************************************************/
static inline void enableGlobalInterrupt(void) {
    _BIS_SR(GIE);
}
static inline void disableGlobalInterrupt(void) {
    _BIC_SR(GIE);
}
static inline void enableSysTimerInterrupt(void) {
    TA1CCTL0 |= CCIE;
}
static inline void disableSysTimerInterrupt(void) {
    TA1CCTL0 &= ~CCIE;
}
static inline void restoreSysTimerInterrupt(uint16_t val) {
    TA1CCTL0 &= ~CCIE; // clear the value
    TA1CCTL0 |= val;   // set to val
}
static inline uint16_t getAndDisableSysTimerInterrupt(void) {
    volatile uint16_t current = TA1CCTL0; // read reg
    current  &= CCIE;  // get current interrupt setting
    TA1CCTL0 &= ~CCIE; // disable interrupt
    return current;    // return interrupt setting
}

/*******************************************************************************
*  Polling Delays
*******************************************************************************/
void secDelay(uint8_t secCount);
void ms1Delay(uint8_t msCount);
void us10Delay(uint8_t us10);

/*******************************************************************************
* main.c
*******************************************************************************/
uint8_t main_copyBootInfo (uint8_t *bufP); 
void main_accessBootInfo(const uint8_t **bufPP, uint8_t *lengthP); 

/*******************************************************************************
* sysExec.c
*******************************************************************************/
void sysExec_exec(void);
void sysExec_startRebootCountdown(uint8_t *keysP);
void sysExec_doReboot(void);
void sysError(void);

/*******************************************************************************
* utils.c
*******************************************************************************/
unsigned int gen_crc16(const unsigned char *data, unsigned int size);

// WDTPW+WDTCNTCL+WDTSSEL
// 1 second time out, uses ACLK
#define WATCHDOG_TICKLE() (WDTCTL = WDT_ARST_1000)
#define WATCHDOG_STOP() (WDTCTL = WDTPW | WDTHOLD)

/*******************************************************************************
* modemCmd.c
*******************************************************************************/

/**
 * \typedef modemCmdWriteData_t 
 * \brief Container to pass parmaters to the modem command write 
 *        function.
 */
typedef struct modemCmdWriteData_s {
    modem_command_t cmd;         /**< the modem command */
    MessageType_t payloadMsgId;  /**< the payload type (Cascade message type) */
    uint8_t *payloadP;           /**< the payload pointer (if any) */
    uint16_t payloadLength;      /**< size of the payload in bytes */
    uint16_t payloadOffset;      /**< for receiving partial data */
    bool statusOnly;             /**< only perform status retrieve from modem - no cmd */
} modemCmdWriteData_t;

/**
 * \typedef modemCmdReadData_t 
 * \brief Container to read the raw response returned from the 
 *        modem as a result of sending it a command. 
 */
typedef struct modemCmdReadData_s {
    modem_command_t modemCmdId;    /**< the cmd we are sending to the modem */
    bool valid;                    /**< indicates that the response is correct (crc passed, etc) */
    uint8_t *dataP;                /**< the pointer to the raw buffer */
    uint16_t lengthInBytes;        /**< the length of the data in the buffer */
} modemCmdReadData_t;

/**
 * \def OTA_PAYLOAD_BUF_LENGTH
 * \brief Specify the size of our OTA buffer where the payload 
 *        portion of received OTA messages will be copied.
 */
#define OTA_PAYLOAD_BUF_LENGTH ((uint16_t)512)

/**
 * \def OTA_RESPONSE_LENGTH
 * \brief Specify the size of an OTA response.  It is a constant 
 *        value
 */
#define OTA_RESPONSE_LENGTH ((uint8_t)48)

/**
 * \def OTA_RESPONSE_HEADER_LENGTH 
 * \brief Define the header length of an OTA response message 
 */
#define OTA_RESPONSE_HEADER_LENGTH ((uint8_t)16)

/**
 * \def OTA_RESPONSE_DATA_LENGTH 
 * \brief Define the data length of an OTA response message. The
 *        data follows the header in the message.
 */
#define OTA_RESPONSE_DATA_LENGTH ((uint8_t)32)

/**
 * \typedef otaResponse_t
 * \brief Define a container to hold a partial OTA response.
 */
typedef struct otaResponse_s {
    uint8_t *buf;                         /**< A buffer to hold one OTA message */
    uint16_t lengthInBytes;               /**< how much valid data is in the buf */
    uint16_t remainingInBytes;            /**< how much remaining of the total OTA */
}otaResponse_t;

void modemCmd_exec(void);
void modemCmd_init(void);
bool modemCmd_write(modemCmdWriteData_t *writeCmdP);
void modemCmd_read(modemCmdReadData_t *readDataP);
bool modemCmd_isError(void);
bool modemCmd_isBusy(void);
void modemCmd_pollUart(void);
void uart_tx_test(void);

/*******************************************************************************
* modemPower.c
*******************************************************************************/
void modemPower_exec(void);
void modemPower_init(void);
void modemPower_restart(void);
void modemPower_powerDownModem(void);
bool modemPower_isModemOn(void);
uint16_t modemPower_getModemUpTimeInSysTicks(void); 
bool modemPower_isModemOnError(void);

/*******************************************************************************
* modemMgr.c
*******************************************************************************/
void modemMgr_exec(void);
void modemMgr_init(void);
bool modemMgr_grab(void);
bool modemMgr_isModemUp(void);
bool modemMgr_isModemUpError(void);
void modemMgr_sendModemCmdBatch(modemCmdWriteData_t *cmdWriteP);
void modemMgr_stopModemCmdBatch(void);
bool modemMgr_isModemCmdComplete(void);
bool modemMgr_isModemCmdError(void);
void modemMgrRelease(void);
void modemMgr_restartModem(void);
bool modemMgr_isAllocated(void);
void modemMgr_release(void);
bool modemMgr_isReleaseComplete(void);
otaResponse_t* modemMgr_getLastOtaResponse(void);
bool modemMgr_isLinkUp(void);
bool modemMgr_isLinkUpError(void);
uint8_t modemMgr_getNumOtaMsgsPending(void);
uint16_t modemMgr_getSizeOfOtaMsgsPending(void);
uint8_t* modemMgr_getSharedBuffer(void);

/*******************************************************************************
* msgOta.c
*******************************************************************************/
/**
 * \typedef fwUpdateResult_t
 * \brief Specify the different status results that the firmware
 *        update state machine can exit with.
 */
typedef enum fwUpdateResult_e {
    RESULT_NO_FWUPGRADE_PERFORMED =  0,
    RESULT_DONE_SUCCESS           =  1,
    RESULT_DONE_ERROR             = -1,
} fwUpdateResult_t;

void otaMsgMgr_exec(void);
void otaMsgMgr_init(void);
void otaMsgMgr_getAndProcessOtaMsgs(bool sos);
bool otaMsgMgr_isOtaProcessingDone(void);
fwUpdateResult_t otaMsgMgr_getFwUpdateResult(void);

/*******************************************************************************
* time.c
*******************************************************************************/
void timerA0_0_init_for_sys_tick(void);
bool timerA0_0_check_for_sys_tick(void);
void timerA0_0_init_for_sleep_tick(void);
__interrupt void ISR_Timer0_A0(void);
uint32_t getSysTicksSinceBoot(void);
static inline void timerA0_0_halt(void) {
    TA0CCTL0 &= ~CCIFG;
    TA0CTL = 0;
}

/*******************************************************************************
* hal.c
*******************************************************************************/
void clock_init(void);
void uart_init(void);
void pin_init(void);

/*******************************************************************************
* flash.c
*******************************************************************************/
/**
 * \typedef fwCopyResult_t
 * \brief Result from copy of flash backup image to 
 *        flash main image.
 */
typedef enum fwCopyResult_e {
    FW_COPY_SUCCESS             =  0,
    FW_COPY_ERR_NO_BACKUP_IMAGE = -1,
    FW_COPY_ERR_BAD_BACKUP_CRC  = -2,
    FW_COPY_ERR_COPY_FAILED     = -3,
    FW_COPY_ERR_BAD_MAIN_CRC    = -4,
} fwCopyResult_t;

void msp430Flash_erase_segment(uint8_t *flashSectorAddrP);
void msp430Flash_write_bytes(uint8_t *flashP, uint8_t *srcP, uint16_t num_bytes);
void msp430Flash_zeroAppResetVector(void);
fwCopyResult_t msp430Flash_moveAndVerifyBackupToApp(void);

/*******************************************************************************
* For Firmware Upgrade Support
*******************************************************************************/

/**
 * \def FLASH_UPGRADE_KEY1
 * \def FLASH_UPGRADE_KEY2
 * \def FLASH_UPGRADE_KEY3
 * \def FLASH_UPGRADE_KEY4
 * \brief These keys are used to validate the OTA firmware
 *        upgrade command.
 */
#define FLASH_UPGRADE_KEY1 ((uint8_t)0x31)
#define FLASH_UPGRADE_KEY2 ((uint8_t)0x41)
#define FLASH_UPGRADE_KEY3 ((uint8_t)0x59)
#define FLASH_UPGRADE_KEY4 ((uint8_t)0x26)

/*******************************************************************************
* appRecord.c
*******************************************************************************/
bool appRecord_initAppRecord(void);
bool appRecord_checkForValidAppRecord(void);
bool appRecord_checkForNewFirmware(void);
bool appRecord_updateFwInfo(bool newFwIsReady, uint16_t newFwCrc);
bool appRecord_getNewFirmwareInfo(bool *newFwReadyP, uint16_t *newFwCrcP);
void appRecord_erase();
#if 0
void appRecord_test(void);
#endif

/*******************************************************************************
* bootRecord.c
*******************************************************************************/
#define BLR_LOCATION ((uint8_t *)0x1080)  // INFO B
#define BLR_MAGIC ((uint16_t)0x1234)

typedef struct bootloaderRecord_s {
    uint16_t magic;
    uint16_t bootRetryCount;
    uint16_t crc16;
} bootloaderRecord_t;

void bootRecord_initBootloaderRecord(void);
int bootRecord_getBootloaderRecordCount(void);
void bootRecord_incrementBootloaderRecordCount(void);

