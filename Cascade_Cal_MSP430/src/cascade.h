/** 
 * @file cascade.h
 * \n Source File
 * \n Cascade MSP430 Firmware
 * 
 * \brief MSP430 System wide header file for Cascade firmware. 
 *        Contains function prototypes, MACROS, and data
 *        definitions for all "C" modules in the Cascade
 *        firmware.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "msp430.h"
#include "msp430g2955.h"
#include "RTC_Calendar.h"
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
#define FW_VERSION_MAJOR ((uint8_t)0x81)

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
#define GET_SYSTEM_TICK() ((sys_tick_t)getSecondsSinceBoot());
// Return the number of elapsed seconds
#define GET_ELAPSED_TIME_IN_SEC(x) (((sys_tick_t)getSecondsSinceBoot())-(sys_tick_t)(x))

// Used for testing if running the sys tick at faster then
// the standard 1 second interval.  For normal operation, set to 1.
#define TIME_SCALER ((uint8_t)1)

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
 * \brief Macro for 30 seconds
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
 * \def SECONDS_PER_DAY
 * \brief Macro to specify seconds per day
 */
#define SECONDS_PER_DAY ((uint32_t)86400)
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
#define LS_VCC               BIT0
#define UART_RXD             BIT5
#define UART_TXD             BIT4
#define GSM_STATUS           BIT4
#define GSM_EN               BIT6
#define GSM_DCDC             BIT7
#define NFC_CHIP_POWER       BIT3
#define DEBUG_P2_3           BIT3
#define DEBUG_P2_4           BIT4
#define X_OUT_P2_7           BIT7
#define X_IN_P2_6            BIT6
#define JIG_DETECT_P2_0      BIT0
#define JIG_YELLOW_LED_P2_1  BIT1
#define JIG_RED_LED_P2_3     BIT3
#define JIG_GREEN_LED_P2_4   BIT4

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
uint8_t getLastRebootReason(void);

/*******************************************************************************
* sysExec.c
*******************************************************************************/
/**
 * \def REBOOT_KEY1
 * \def REBOOT_KEY2
 * \def REBOOT_KEY3
 * \def REBOOT_KEY4
 * \brief These keys are used to validate the OTA reset command.
 */
#define REBOOT_KEY1 ((uint8_t)0xAA)
#define REBOOT_KEY2 ((uint8_t)0x55)
#define REBOOT_KEY3 ((uint8_t)0xCC)
#define REBOOT_KEY4 ((uint8_t)0x33)

void sysExec_exec(void);
bool sysExec_startRebootCountdown(uint8_t *keysP);
void sysExec_doReboot(void);
void sysError(void);
void sysExec_sendDebugDataToUart(void);

/*******************************************************************************
* utils.c
*******************************************************************************/
typedef struct timeCompare_s {
    uint8_t hoursA;
    uint8_t minutesA;
    uint8_t secondsA;
    uint8_t hoursB;
    uint8_t minutesB;
    uint8_t secondsB;
    uint32_t timeDiffInSeconds;
} timeCompare_t;

bool isBcdMinSecValValid(uint8_t bcdVal);
bool isBcdHour24Valid(uint8_t bcdVal);
unsigned int gen_crc16(const unsigned char *data, unsigned int size);
unsigned int gen_crc16_2buf(const unsigned char *data1, unsigned int size1, const unsigned char *data2, unsigned int size2);
uint32_t timeInSeconds(uint8_t hours, uint8_t minutes, uint8_t seconds);
void calcTimeDiffInSeconds(timeCompare_t *timeCompareP);

/*******************************************************************************
* modemCmd.h
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
 *        value.  It consists of the header and the data.  The
 *        header is 16 bytes and the data is 32 bytes.  
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
bool modemCmd_isResponseReady(void);
bool modemCmd_isError(void);
bool modemCmd_isBusy(void);

/*******************************************************************************
* modemLink.h
*******************************************************************************/
void modemLink_exec(void);
void modemLink_init(void);
void modemLink_restart(void);
void modemLink_shutdownModem(void);
bool modemLink_isModemUp(void);
uint16_t modemLink_getModemUpTimeInSecs(void);
bool modemLink_isModemUpError(void);

/*******************************************************************************
* modemMgr.c
*******************************************************************************/

/**
 * \typedef msgHeader_t
 * \brief Define the structure of the header that sits on top of
 *        of all outbound messages.
 */
typedef struct __attribute__((__packed__))msgHeader_s {
    uint8_t payloadStartByte;       /**< 0 */
    uint8_t payloadMsgId;           /**< 1 */
    uint8_t productId;              /**< 2 */
    uint8_t GMTsecond;              /**< 3 */
    uint8_t GMTminute;              /**< 4 */
    uint8_t GMThour;                /**< 5 */
    uint8_t GMTday;                 /**< 6 */
    uint8_t GMTmonth;               /**< 7 */
    uint8_t GMTyear;                /**< 8 */
    uint8_t fwMajor;                /**< 9 */
    uint8_t fwMinor;                /**< 10 */
    uint8_t daysActivatedMsb;       /**< 11 */
    uint8_t daysActivatedLsb;       /**< 12 */
    uint8_t storageWeek;            /**< 14 */
    uint8_t storageDay;             /**< 13 */
    uint8_t reserve1;               /**< 15 */
} msgHeader_t;

void modemMgr_exec(void);
void modemMgr_init(void);
bool modemMgr_grab(void);
bool modemMgr_isModemUp(void);
bool modemMgr_isModemUpError(void);
void modemMgr_sendModemCmdBatch(modemCmdWriteData_t *cmdWriteP);
void modemMgr_stopModemCmdBatch(void);
bool modemMgr_isModemCmdComplete(void);
bool modemMgr_isModemCmdError(void);
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
void modemMgr_getImei(uint8_t *bufP);

/*******************************************************************************
* msgData.c
*******************************************************************************/
void dataMsgMgr_exec(void);
void dataMsgMgr_init(void);
bool dataMsgMgr_isSendMsgActive(void);
bool dataMsgMgr_sendDataMsg(MessageType_t msgId, uint8_t *dataP, uint16_t lengthInBytes);
bool dataMsgMgr_sendTestMsg(MessageType_t msgId, uint8_t *dataP, uint16_t lengthInBytes);
bool dataMsgMgr_sendDailyLogs(void);

/*******************************************************************************
* msgOta.c
*******************************************************************************/
void otaMsgMgr_exec(void);
void otaMsgMgr_init(void);
void otaMsgMgr_getAndProcessOtaMsgs(void);
bool otaMsgMgr_isOtaProcessingDone(void);

/*******************************************************************************
* msgOtaUpgrade.c
*******************************************************************************/

/**
 * \typedef fwUpdateResult_t
 * \brief Specify the different status results that the firmware
 *        update state machine can exit in.
 */
typedef enum fwUpdateResult_e {
    RESULT_NO_FWUPGRADE_PERFORMED =  0,
    RESULT_DONE_SUCCESS           =  1,
    RESULT_DONE_ERROR             = -1,
} fwUpdateResult_t;

fwUpdateResult_t otaUpgrade_processOtaUpgradeMessage(void);
fwUpdateResult_t otaUpgrade_getFwUpdateResult(void);
uint16_t otaUpgrade_getFwMessageCrc(void);
uint16_t otaUpgrade_getFwCalculatedCrc(void);
uint16_t otaUpgrade_getFwLength(void);
uint8_t otaUpgrade_getErrorCode(void);

/*******************************************************************************
* msgDebug.c
*******************************************************************************/

typedef struct eventSysInfo_s {
    uint8_t storageTime_seconds;       /**< Current storage time - sec  */
    uint8_t storageTime_minutes;       /**< Current storage time - min */
    uint8_t storageTime_hours;         /**< Current storage time - hour */
    uint8_t storageTime_dayOfWeek;     /**< Current storage time - day */
    uint8_t storageTime_week;          /**< Current storage time - week */
    uint16_t daysActivated;            /**< Total days activated */
    uint16_t currentMinuteML;          /**< Running sum for last minute */
    uint32_t currentHourML;            /**< Running sum for last hour */
    uint16_t dailyLiters;              /**< Running sum for last day */
} eventSysInfo_t;

typedef enum debugEvents_e {
    EVENT_RESET = 0x100,
    EVENT_SYS_INFO,
    EVENT_GMT_UPDATE,
    EVENT_CLOCK_ALIGNMENT,
    EVENT_ACTIVATION,
    EVENT_REDFLAG,
    EVENT_MODEM_TIMEOUT
} debugEvents_t;

void dbgMsgMgr_init(void);
void dbgMsgMgr_sendDebugMsg(MessageType_t msgId, uint8_t *dataP, uint16_t lengthInBytes);

/*******************************************************************************
* msgData.c
*******************************************************************************/
/**
 * \typedef dataMsgState_t
 * \brief Specify the states for sending a data msg to the 
 *        modem.
 */
typedef enum dataMsgState_e {
    DMSG_STATE_IDLE,
    DMSG_STATE_GRAB,
    DMSG_STATE_WAIT_FOR_MODEM_UP,
    DMSG_STATE_SEND_MSG,
    DMSG_STATE_SEND_MSG_WAIT,
    DMSG_STATE_WAIT_FOR_LINK,
    DMSG_STATE_PROCESS_OTA,
    DMSG_STATE_PROCESS_OTA_WAIT,
    DMSG_STATE_RELEASE,
    DMSG_STATE_RELEASE_WAIT,
} dataMsgState_t;

/**
 * \typedef dataMsgSm_t
 * \brief Define a contiainer to hold the information needed by 
 *        the data message module to perform sending a data
 *        command to the modem.
 *  
 * \note To save memory, this object can potentially be a common
 *       object that all clients use because only one client
 *       will be using the modem at a time?
 */
typedef struct dataMsgSm_s {
    dataMsgState_t dataMsgState;  /**< current data message state */
    modemCmdWriteData_t cmdWrite; /**< the command info object */
    uint8_t modemResetCount;      /**< for error recovery, count times modem is power cycled */
    bool sendCmdDone;             /**< flag to indicate sending the current command to modem is complete */
    bool allDone;                 /**< flag to indicate send session is complete and modem is off */
    bool connectTimeout;          /**< flag to indicate the modem was not able to connect to the network */
    bool commError;               /**< flag to indicate an modem UART comm error occured - not currently used - can remove */
} dataMsgSm_t;

void dataMsgSm_init(void);
void dataMsgSm_initForNewSession(dataMsgSm_t *dataMsgP);
void dataMsgSm_sendAnotherDataMsg(dataMsgSm_t *dataMsgP);
void dataMsgSm_stateMachine(dataMsgSm_t *dataMsgP);

/*******************************************************************************
* time.c
*******************************************************************************/
/**
 * \typedef timePacket_t 
 * \brief Specify a structure to hold time data that will be 
 *        sent as part of the final assembly message.
 */
typedef struct  __attribute__((__packed__))timePacket_s {
    uint8_t second;
    uint8_t minute;
    uint8_t hour24;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} timePacket_t;

void timerA1_init(void);
timePacket_t* getBinTime(void);
timePacket_t* getBcdTime(void);
uint8_t bcd_to_char(uint8_t bcdValue);
uint8_t char_to_bcd(uint8_t binVal);
uint32_t getSecondsSinceBoot(void);
#if 0
void calibrateLoopDelay (void);
#endif

// WDTPW+WDTCNTCL+WDTSSEL
// 1 second time out, uses ACLK
#define WATCHDOG_TICKLE() (WDTCTL = WDT_ARST_1000)
#define WATCHDOG_STOP() (WDTCTL = WDTPW | WDTHOLD)

/*******************************************************************************
* storage.c
*******************************************************************************/
void storageMgr_init(void);
void storageMgr_exec(void);
void storageMgr_overrideUnitActivation(bool flag);
uint16_t storageMgr_getDaysActivated(void);
void storageMgr_resetRedFlag(void);
bool storageMgr_getRedFlagConditionStatus(void);
void storageMgr_resetRedFlagAndMap(void);
void storageMgr_resetWeeklyLogs(void);
void storageMgr_setStorageAlignmentTime(uint8_t alignSecond, uint8_t alignMinute, uint8_t alignHour24);
void storageMgr_setTransmissionRate(uint8_t transmissionRateInDays);
uint16_t storageMgr_getNextDailyLogToTransmit(uint8_t **dataPP);
void storageMgr_sendDebugDataToUart(void);
uint8_t storageMgr_getStorageClockInfo(uint8_t *bufP);
uint8_t storageMgr_prepareMsgHeader(uint8_t *dataPtr, uint8_t payloadMsgId);

/*******************************************************************************
* flowSensor.c
*******************************************************************************/

/**
 * \def USE_TIMER_FOR_PULSE_COUNTING
 * \brief If macro is defined, then timer B is used to count the 
 *        flow pulses as opposed to using interrupts. The flow
 *        pulses clock the timerB via the external timer input
 *        TB0CLK.
 */
#define USE_TIMER_FOR_PULSE_COUNTING 1

void flowSensor_init(void);
void flowSensor_exec(void);
uint16_t flowSensor_getLastMeasFlowRateInML(void);
uint16_t flowSensor_getMaxFlowRateInML(void);
void flowSensor_sendDebugDataToUart(void);
uint32_t flowSensor_getSecondsOfFlow(void);
uint32_t flowSensor_getTotalPulseCount(void);
uint16_t flowSensor_getLastPulseCount(void);

/*******************************************************************************
* flowEvents.c
*******************************************************************************/
void flowEvent_init(void);
void flowEvent_exec(void);
void flowEvent_setConstants(uint16_t slope, int16_t offset);

/*******************************************************************************
* hal.c
*******************************************************************************/
void hal_sysClockInit(void);
void hal_uartPinInit(void);
void hal_pinInit(void);
bool hal_testJigDetected(void);
void hal_yellowLedOn(void);
void hal_yellowLedOff(void);
void hal_redLedOn(void);
void hal_redLedOff(void);
void hal_greenLedOn(void);
void hal_greenLedOff(void);

/*******************************************************************************
* flash.c
*******************************************************************************/
void msp430Flash_erase_segment(uint8_t *flashSectorAddrP);
void msp430Flash_write_bytes(uint8_t *flashP, uint8_t *srcP, uint16_t num_bytes);
void msp430Flash_write_int16(uint8_t *flashP, uint16_t val16);
void msp430Flash_write_int32(uint8_t *flashP, uint32_t val32);
#if 0
void msp430flash_test(void);
#endif

/*******************************************************************************
* flashParam.c
*******************************************************************************/

void flashParam_init(void);
bool flashParam_getFlowConstants(uint16_t *slopeX1000P, int16_t *offsetX1000P);
void flashParam_setFlowConstants(uint16_t slopeX1000P, int16_t offsetX1000P);
void flashParam_test(void);

/*******************************************************************************
* cqueue.c
*******************************************************************************/
/**
 * \struct CQUEUE_S 
 * \brief structure to manage a circular queue of 
 *       objects
 * \typedef CQUEUE_T
 * \brief Create typedef to the structure CQUEUE_S
 */
typedef struct CQUEUE_S {
    uint8_t *bufP;         /**< pointer to first address of circular */
    uint32_t size;         /**< size of queue in terms of objects */
    uint32_t objSize;      /**< size of each object in terms of bytes */
    uint32_t tail_index;   /**< where to add the next entry (submitter) */
    uint32_t head_index;   /**< where to remove the oldest entry (consumer) */
    uint32_t num_entries;  /**< number of filled entries currently in queue */
} CQUEUE_T;

void cqueue_init(CQUEUE_T *cqP, uint8_t *bufP, uint32_t size, uint32_t objSize);
void cqueue_reset(CQUEUE_T *cqP);
uint32_t cqueue_write(CQUEUE_T *cqP, uint8_t *dataP, uint32_t num_objs);
uint32_t cqueue_read(CQUEUE_T *cqP, uint8_t *dataP, uint32_t num_objs);
uint32_t cqueue_getNumFilledEntries(CQUEUE_T *cqP);
bool cqueue_isQueueFull(CQUEUE_T *cqP);

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
* luhn.c
*******************************************************************************/
uint64_t luhn(uint64_t id);
uint8_t imeiToAscii(uint64_t id, uint8_t *bufP);
uint64_t imeiArrayToInt(uint8_t *bufP);

/*******************************************************************************
* i2c.c
*******************************************************************************/
typedef int16_t error_t;
#define ERROR_SUCCESS     0
#define ERROR_I2C_FAILED -1

error_t i2c_read(unsigned char *buf, unsigned int len, unsigned char slave, unsigned char addr);
error_t i2c_write(unsigned char slave, unsigned char addr, const unsigned char *data, unsigned int len);

/*******************************************************************************
* nfc.c
*******************************************************************************/
error_t nfc_programTag();

