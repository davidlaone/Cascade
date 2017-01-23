/** 
 * @file flashParam.c
 * \n Source File
 * \n Cascade MSP430 Firmware
 * 
 * \brief Routines to support storing and retrieving parameters 
 *        to/from flash INFO "D" section.  These parameters must
 *        be "update-able", and the values must be retained
 *        across reboots.
 */

#include "cascade.h"

/***************************
 * Module Data Definitions
 **************************/

/**
 * \def PARAM_LOCATION
 * \brief Where in flash the parameter data base is located. 
 *        It is located in the Flash INFO D section.
 */
#define PARAM_LOCATION ((uint8_t *)0x1000)

/**
 * \def PARAM_MAGIC
 * \brief Used as a known pattern to perform a "quick" verify 
 *        that the data structure is initialized in flash.
 */
#define PARAM_MAGIC ((uint16_t)0x1234)

/**
 * \def PARAM_VERSION 
 * \brief Version number of structure format.
 */
#define PARAM_VERSION ((uint16_t)0x0)

/**
 * \typedef paramRecord_T 
 * \brief This is the data structure definition for the 
 *        parameter data base.
 */
typedef struct paramRecord_s {
    uint16_t magic;         /**< A known pattern for "quick" test of structure validity */
    uint16_t recordLength;  /**< Length of structure */
    uint16_t version;       /**< Version of structure format */
    uint16_t slopeX1000;    /**< Parameter */
    int16_t offsetX1000;    /**< Parameter */
    uint16_t crc16;         /**< Used to validate the data */
} paramRecord_t;

/****************************
 * Module Data Declarations
 ***************************/

/*************************
 * Module Prototypes
 ************************/

static void flashParam_initParamRecord(void);
static void flashParam_checkAndInitRecord(void);
static bool flashParam_checkForValidParamRecord(void);

/***************************
 * Module Public Functions
 **************************/

/**
* \brief Check if the parameter data base structure is valid or
*        not.  If not valid, initialize it.  To validate, the
*        CRC stored in the structure is compared against the
*        calculated CRC value.
* \ingroup PUBLIC_API
*/
void flashParam_init(void) {
    flashParam_checkAndInitRecord();
}
/**
* \brief Retrieve the water flow volume calculation constants 
*        from the parameter data base (volume = mx+b).
* \ingroup PUBLIC_API
* 
* @param slopeX1000P 16 bit value representing slope constant x
*                    1000.
* @param offsetX1000P 16 bit value representing offset constant x
*                     1000.
* 
* @return bool - Returns true if the data in the parameter data 
*         base is valid.
*/
bool flashParam_getFlowConstants(uint16_t *slopeX1000P, int16_t *offsetX1000P) {
    bool recordStatus = false;
    paramRecord_t *paramP = (paramRecord_t *)PARAM_LOCATION;
    // Verify the data structure.
    if (flashParam_checkForValidParamRecord()) {
        *slopeX1000P = paramP->slopeX1000;
        *offsetX1000P = paramP->offsetX1000;
        recordStatus = true;
    } else {
        *slopeX1000P = 0;
        *offsetX1000P = 0;
    }
    return recordStatus;
}

/**
* \brief Set the water flow volume calculation constants in the 
*        parameter data base.
* \ingroup PUBLIC_API
* 
* @param slopeX1000P 16 bit value representing slope constant x 
*                    1000.
* @param offsetX1000P 16 bit value representing offset constant 
*                     x 1000.
*/
void flashParam_setFlowConstants(uint16_t slopeX1000P, int16_t offsetX1000P) {
    paramRecord_t paramRecord;
    uint8_t retryCount = 0;
    bool error = false;
    // Retry up to four times to write the Param Record.
    do {
        // Create fresh RAM version
        memset(&paramRecord, 0, sizeof(paramRecord_t));
        paramRecord.magic = PARAM_MAGIC;
        paramRecord.recordLength = sizeof(paramRecord_t);
        paramRecord.version = PARAM_VERSION;
        paramRecord.slopeX1000 = slopeX1000P;
        paramRecord.offsetX1000 = offsetX1000P;
        // Calculate CRC on RAM version
        paramRecord.crc16 = gen_crc16((const unsigned char *)&paramRecord, (sizeof(paramRecord_t) - sizeof(uint16_t)));
        // Erase Flash Version
        msp430Flash_erase_segment(PARAM_LOCATION);
        // Copy RAM version to Flash version
        msp430Flash_write_bytes(PARAM_LOCATION, (uint8_t *)&paramRecord, sizeof(paramRecord_t));
        // Final check
        if (!flashParam_checkForValidParamRecord()) {
            error = true;
        } else {
            error = false;
        }
    } while (error && (retryCount++ < 4));
}

/***************************
 * Module Private Functions
 **************************/

/**
* \brief Checks if the parameter data base if valid by checking 
*        the stored CRC value against the calculated CRC value.
*        If it is not valid, the parameter data base structure
*        is initialized with default values.
*/
static void flashParam_checkAndInitRecord(void) {
    if (!flashParam_checkForValidParamRecord()) {
        // If the record is not found, write one.
        flashParam_initParamRecord();
    }
}

/**
* \brief Determine if a valid Parameter Record is located in the
*        INFO D section. The recordLength variable is used to
*        identify where the CRC is located. This allows the
*        structure to grow and have elements added, and future
*        firmware versions will still be able to verify its
*        contents. But the original elements must remain in the
*        structure at their current defined location (except for
*        the CRC location which always at the end - 2).
* 
* @return bool Returns true if the record is valid.  False 
*         otherwise.
*/
static bool flashParam_checkForValidParamRecord(void) {
    bool validFlag = false;
    paramRecord_t *paramP = (paramRecord_t *)PARAM_LOCATION;
    if (paramP->magic == PARAM_MAGIC) {
        // Locate the offset to the CRC in the structure based on the
        // stored record length of the paramRecord. This is to ensure
        // backward compatibility with future versions.
        uint8_t crcOffset = paramP->recordLength - sizeof(uint16_t);
        // Calculate CRC
        uint16_t calcCrc = gen_crc16(PARAM_LOCATION, crcOffset);
        // Access stored CRC based on stored record length, not structure element
        // because structure may be added to in the future.
        uint16_t *storedCrcP = (uint16_t *)(((uint8_t *)paramP) + crcOffset);
        uint16_t storedCrc =  *storedCrcP;
        // Compare calculated CRC to the CRC stored in the structure
        if (calcCrc == storedCrc) {
            validFlag = true;
        }
    }
    return validFlag;
}

/**
* \brief Initialize the parameter data base structure in the 
*        flash INFO segment.  Initializes the parameters to
*        default values, and sets the MAGIC and CRC values.
*/
static void flashParam_initParamRecord(void) {
    paramRecord_t paramRecord;
    uint8_t retryCount = 0;
    bool error = false;
    // Retry up to four times to write the Param Record.
    do {
        // Zero RAM version and update its parameters
        memset(&paramRecord, 0, sizeof(paramRecord_t));
        paramRecord.magic = PARAM_MAGIC;
        paramRecord.recordLength = sizeof(paramRecord_t);
        paramRecord.version = PARAM_VERSION;
        paramRecord.crc16 = gen_crc16((const unsigned char *)&paramRecord, (sizeof(paramRecord_t) - sizeof(uint16_t)));

        // Erase flash version
        msp430Flash_erase_segment(PARAM_LOCATION);
        // Write RAM version to flash version
        msp430Flash_write_bytes(PARAM_LOCATION, (uint8_t *)&paramRecord, sizeof(paramRecord_t));

        // Final check
        if (!flashParam_checkForValidParamRecord()) {
            error = true;
        } else {
            error = false;
        }

    } while (error && (retryCount++ < 4));
}

#if 0
uint16_t writeSlopeX1000 = 0x1234;
int16_t writeOffsetX1000 = 0x5678;
uint16_t readSlopeX1000 = 0;
int16_t readOffsetX1000 = 0;
uint8_t flashParamTestError = 0;
void flashParam_test(void) {
    uint8_t i;
    // Setup I/O for timing debug
    P3DIR |= RXD + TXD; // P3.5 = RXD, P3.4=TXD
    for (i = 0; i < 16; i++) {
        if (1) {
            flashParam_setFlowConstants(writeSlopeX1000, writeOffsetX1000);
        }
        if (1) {
            P3OUT |= TXD;
            flashParam_getFlowConstants(&readSlopeX1000, &readOffsetX1000);
            P3OUT &= ~TXD;
            if (writeSlopeX1000 != readSlopeX1000) {
                flashParamTestError++;
            }
            if (writeOffsetX1000 != readOffsetX1000) {
                flashParamTestError++;
            }
        }
        writeSlopeX1000 ^= 0xFFFF;
        writeOffsetX1000 ^= 0xFFFF;
    }
}
#endif



