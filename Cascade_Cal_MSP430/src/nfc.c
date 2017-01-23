/** 
 * @file nfcTag.c
 * \n Source File
 * \n Cascade MSP430 Firmware
 * 
 * \brief Support for programming the NFC tag.
 */

/***************************
 * Module Data Definitions
 **************************/

#include "cascade.h"

/** Number of times to retry I2C operation */
#define	_I2C_ATTEMPTS						3

/** Time to wait before I2C read operations (in milliseconds) */
#define	_I2C_READ_DELAY						100

/** Time to wait before and after I2C write operations (in milliseconds) */
#define	_I2C_WRITE_DELAY					500

/** I2C address of NFC tag */
#define	_I2C_ADDRESS						0x55

/** Block size of NFC tag (in bytes) */
#define	_BLOCK_SIZE							16

/** Buffer length (i.e. total length of NDEF data plus zero terminator) (in bytes) */
#define	_NDEF_SIZE							(2 * _BLOCK_SIZE)

/** Address of control block */
#define	_CTRL_ADDR							0

/** Address of first NDEF block */
#define	_NDEF_ADDR							1

/** Position of I2C address byte in control block */
#define	_I2C_ADDR_POS						0

/** IMEI format string */
#define	_IMEI_FORMAT						"%015llu"

/** NDEF terminator TLV byte */
#define	_NDEF_TERMINATOR					0xFE

/***************************
 * Module Data Declarations
 **************************/

/**
 * Control block:
 * 00: I2C address: 0
 * 01-06: Serial number: all 0
 * 07-09: Internal: all 0
 * 10-11: Static lock bytes: all locked
 * 12-15: Capability Container:
 *	byte 0: magic number (0xE1)
 *	byte 1: mapping version (1.0), unlimited read, write not allowed
 *	byte 2: memory size: 112 bytes
 *	byte 3: Read multiple blocks & Inventory page read not supported
 */
static const unsigned char _ctrl_block[] = {
    0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00,
    0xff, 0xff,
    0xe1, 0x10, 0x0e, 0x00
};

/**
 * Control block update mask (update static lock bytes & capability container).
 */
static const unsigned char _ctrl_block_mask[] = {
    0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00,
    0xff, 0xff,
    0xff, 0xff, 0xff, 0xff
};

/**
 * NDEF message:
 * 00: NDEF message start
 * 01: NDEF message length
 * -- NDEF record --
 * 02: MB (start of NDEF Message), ME (end of NDEF Message), SF (short format) flags set, well-known type
 * 03: length of record type: 1 byte
 * 04: length of payload: 15 bytes
 * 05: record type: plain text
 * 06: status byte: 2-byte language code
 * 07-08: language: English
 *  
 * Assumptions: 
 * The 02,'e'&'n' are part of the payload.  Hence the payload 
 * length is 15 bytes for the luhn encoded imei number + 3 bytes 
 * = 18 = 0x12 = payload length. 
 * The NDEF message length is four bytes (0xd1,0x01,0x12,0x54) + 
 * 18 (see above) = 22 = 0x16. 
 */
static const unsigned char _ndef_header[] = { 0x03, 0x16, 0xd1, 0x01, 0x12, 0x54, 0x02, 'e', 'n' };

/**
* \var i2cBuf
* @brief Buffer to used to hold i2c write and read data
*/
static uint8_t i2cBuf[_NDEF_SIZE];

/**
* @var scratchBuf
* @brief Buffer used to compose the control block and ndef 
*        message data that will be written to the NFC Tag
*/
static uint8_t scratchBuf[_NDEF_SIZE];

/**************************
 * Module Prototypes
 **************************/

static error_t _update_ctrl(void);
static error_t _update_ndef(void);
static error_t _read(uint8_t *buf, uint8_t len, uint8_t slave, uint8_t addr);
static error_t _write(uint8_t slave, uint8_t addr, const uint8_t *data, uint8_t len);
static void nfc_chip_powerup(void);
static void nfc_chip_powerdown(void);
static delay_500ms(void);
static delay_100ms(void);

/***************************
 * Module Public Functions 
 **************************/

/**
* @brief  Updates the NFC tag (if present) with the device's 
*        IMEI if it hasn't already been updated.
* 
* @return error_t #ERROR_SUCCESS on success; #ERROR_I2C_FAILED 
*         otherwise
*/
error_t nfc_programTag() {
    error_t err;
    WATCHDOG_STOP();
    nfc_chip_powerup();
    err = _update_ctrl();
    if (err != ERROR_SUCCESS) goto end;
    err = _update_ndef();
    if (err != ERROR_SUCCESS) goto end;
end:
    nfc_chip_powerdown();
    WATCHDOG_TICKLE();
    return err;
}

/***************************
 * Module Data Definitions
 **************************/

/**
* @brief Updates the control block of the tag.
* 
* @return error_t #ERROR_SUCCESS on success; #ERROR_I2C_FAILED 
*         otherwise
*/
static error_t _update_ctrl(void) {
    uint8_t i;
    uint8_t j;
    error_t err;
    uint8_t *ctrlTemplateP = scratchBuf;
    int8_t compareResult = 0;

    // Perform read, compare and write
    for (j = 0,compareResult = ~0; (j < 2) && (compareResult != 0); j++) {
        // Read existing control block of NFC Tag
        err = _read(i2cBuf, sizeof(_ctrl_block), _I2C_ADDRESS, _CTRL_ADDR);
        if (err == ERROR_SUCCESS) {
            // Compose expected control block contents
            // Note: Byte 0 read back from NFC is always read back as 0x4.
            // Writing byte 0 sets the NFC's I2C address.
            // Set byte 0 to the NFC's assigned I2C address so it looks like a match.
            // Also it will get programmed correctly if we do write to the control block.
            i2cBuf[_I2C_ADDR_POS] = _I2C_ADDRESS << 1;
            for (i = 0; i < sizeof(_ctrl_block); i++) {
                ctrlTemplateP[i] = (i2cBuf[i] & ~_ctrl_block_mask[i]) | (_ctrl_block[i] & _ctrl_block_mask[i]);
            }
            // Update tag if existing doesn't match expected data
            compareResult = memcmp(i2cBuf, ctrlTemplateP, sizeof(_ctrl_block));
            if (compareResult != 0) {
                err = _write(_I2C_ADDRESS, _CTRL_ADDR, ctrlTemplateP, sizeof(_ctrl_block));
            }
        }
    }

    // If read and write compare does not match, set return err code
    if (compareResult != 0) {
        err = ERROR_I2C_FAILED;
    }

    return err;
}

/**
* @brief  Updates the NDEF blocks of the tag.
* 
* @return error_t #ERROR_SUCCESS on success; #ERROR_I2C_FAILED 
*         otherwise
*/
static error_t _update_ndef(void) {
    uint64_t imei;
    uint8_t i;
    uint8_t j;
    uint8_t pos = 0;
    error_t err = ERROR_SUCCESS;
    int8_t compareResult;
    uint8_t *ndefTemplate = scratchBuf;
    uint8_t imeiBuf[8];

    // Read the imei received from the modem
    modemMgr_getImei(imeiBuf);
    // Convert imei array to a 64bit value
    imei = imeiArrayToInt(imeiBuf);
    // Add the luhn digit
    imei = luhn(imei);

    // Compose the NDEF message
    memset(ndefTemplate, 0, _NDEF_SIZE);
    memcpy(ndefTemplate, _ndef_header, sizeof(_ndef_header));
    pos += sizeof(_ndef_header);
    // pos += snprintf((char *)&ndefTemplate[pos], _NDEF_SIZE - pos, _IMEI_FORMAT, imei);
    // Add the imei value in ASCII form to the buffer
    pos += imeiToAscii(imei, (uint8_t *)&ndefTemplate[pos]);
    ndefTemplate[pos++] = _NDEF_TERMINATOR;

    // Perform read, compare and write
    for (j = 0,compareResult = ~0; (j < 2) && (compareResult != 0); j++) {
        // Read Tag's version of stored NDEF message.
        // Read two blocks of 16 bytes each.
        for (i = 0; i < _NDEF_SIZE / _BLOCK_SIZE; i++) {
            // read one block
            err = _read(&i2cBuf[_BLOCK_SIZE * i], _BLOCK_SIZE, _I2C_ADDRESS, _NDEF_ADDR + i);
            if (err != ERROR_SUCCESS) {
                break;
            }
        }
        if (!err) {
            // Update Tag's version if it doesn't match
            // Write two blocks of 16 bytes each.
            compareResult = memcmp(i2cBuf, ndefTemplate, pos);
            if (compareResult != 0) {
                for (i = 0; i < _NDEF_SIZE / _BLOCK_SIZE; i++) {
                    // Write one block
                    err = _write(_I2C_ADDRESS, _NDEF_ADDR + i, &ndefTemplate[_BLOCK_SIZE * i], _BLOCK_SIZE);
                    if (err != ERROR_SUCCESS) {
                        break;
                    }
                }
            }
        }
    }

    // If read and write compare does not match, set return err code
    if (compareResult != 0) {
        err = ERROR_I2C_FAILED;
    }

    return err;
}

/**
 * Performs a read operation on the I2C bus and retries it if unsuccessful.
 * The task sleeps before each attempt to allow for the slave to initialize.
 * @param buf Read buffer
 * @param len Length of data to read
 * @param slave I2C address of slave device
 * @param addr Register address
 * @return #ERROR_SUCCESS on success; #ERROR_I2C_FAILED otherwise
 */
static error_t _read(uint8_t *buf, uint8_t len, uint8_t slave, uint8_t addr) {
    uint8_t i;
    error_t err;
    for (i = 0; i < _I2C_ATTEMPTS; i++) {
        err = i2c_read(buf, len, slave, addr);
        if (err == ERROR_SUCCESS) {
            delay_100ms();
            break;
        }
        delay_100ms();
    }
    return err;
}

/**
 * Performs a write operation on the I2C bus and retries it if unsuccessful.
 * The task sleeps before each attempt to allow for the slave to initialize.
 * @param slave I2C address of slave device
 * @param addr Register address
 * @param data Data to write
 * @param len Length of data to write
 * @return #ERROR_SUCCESS on success; #ERROR_I2C_FAILED otherwise
 */
static error_t _write(uint8_t slave, uint8_t addr, const uint8_t *data, uint8_t len) {
    uint8_t i;
    error_t err;
    for (i = 0; i < _I2C_ATTEMPTS; i++) {
        err = i2c_write(slave, addr, data, len);
        if (err == ERROR_SUCCESS) {
            delay_500ms();
            break;
        }
        delay_500ms();
    }
    return err;
}

static void nfc_chip_powerup(void) {
    // Set IO output to hi (Powers NFC and Temperature/Humidity Chip).
    P3OUT |= NFC_CHIP_POWER;
    // Give time for chip(s) to power on
    delay_500ms();
}

static void nfc_chip_powerdown(void) {
    // Set IO output to low
    P3OUT &= ~(NFC_CHIP_POWER);
}

static delay_500ms(void) {
    volatile uint8_t i;
    for (i = 0; i < 50; i++) {
        _delay_cycles(10000);
    }
}

static delay_100ms(void) {
    volatile uint8_t i;
    for (i = 0; i < 10; i++) {
        _delay_cycles(10000);
    }
}
