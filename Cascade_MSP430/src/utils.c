/** 
 * @file utils.c
 * \n Source File
 * \n Cascade MSP430 Firmware
 * 
 * \brief Miscellaneous support functions.
 */

#include "cascade.h"

/**
 * \def CRC16
 * \brief The CRC-16-ANSI polynomial constant.
 */
#define CRC16 0x8005

/**
* \brief Utility function to calculate a 16 bit CRC on data in a
*        buffer.
* 
* @param data Pointer to the data buffer to calculate the CRC
*             over
* @param size Length of the data in bytes to calculate over.
* 
* @return unsigned int The CRC calculated value
*/
unsigned int gen_crc16(const unsigned char *data, unsigned int size) {
    volatile unsigned int out = 0;
    volatile int bits_read = 0;
    volatile int bit_flag;

    while (size > 0) {
        bit_flag = out >> 15;
        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits
        /* Increment bit counter: */
        bits_read++;
        if (bits_read > 7) {
            bits_read = 0;
            data++;
            size--;
            WATCHDOG_TICKLE();
        }
        /* Cycle check: */
        if (bit_flag) out ^= CRC16;
    }

    // item b) "push out" the last 16 bits
    unsigned int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if (bit_flag) out ^= CRC16;
    }

    // item c) reverse the bits
    unsigned int crc = 0;
    i = 0x8000;
    unsigned int j = 0x0001;
    for (; i != 0; i >>= 1, j <<= 1) {
        if (i & out) crc |= j;
    }
    return crc;
}

/**
* \brief Utility function to calculate a 16 bit CRC on data 
*        located in two buffers. The first buffer is most likely
*        the opcode buffer, and the second buffer is the data
*        buffer.
* 
* @param data1 Pointer to first buffer
* @param size1 Size of data in bytes for first buffer
* @param data2 Pointer to data in second buffer
* @param size2 Size of data in bytes for second buffer
* 
* @return unsigned int The calculated CRC value
*/
unsigned int gen_crc16_2buf(const unsigned char *data1, unsigned int size1, const unsigned char *data2, unsigned int size2) {
    volatile unsigned int out = 0;
    volatile int bits_read = 0;
    volatile int bit_flag;

    while (size1 > 0) {
        bit_flag = out >> 15;
        /* Get next bit: */
        out <<= 1;
        out |= (*data1 >> bits_read) & 1; // item a) work from the least significant bits
        /* Increment bit counter: */
        bits_read++;
        if (bits_read > 7) {
            bits_read = 0;
            data1++;
            size1--;
            WATCHDOG_TICKLE();
        }
        /* Cycle check: */
        if (bit_flag) out ^= CRC16;
    }

    while (size2 > 0) {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data2 >> bits_read) & 1; // item a) work from the least significant bits

        /* Increment bit counter: */
        bits_read++;
        if (bits_read > 7) {
            bits_read = 0;
            data2++;
            size2--;
            WATCHDOG_TICKLE();
        }
        /* Cycle check: */
        if (bit_flag) out ^= CRC16;
    }

    // item b) "push out" the last 16 bits
    unsigned int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if (bit_flag) out ^= CRC16;
    }

    // item c) reverse the bits
    unsigned int crc = 0;
    i = 0x8000;
    unsigned int j = 0x0001;
    for (; i != 0; i >>= 1, j <<= 1) {
        if (i & out) crc |= j;
    }
    return crc;
}


/**
* \brief Validate a Minute or Second BCD value for a legal 
*        range.
* 
* @param bcdVal The bcd value to check (0-59)
* 
* @return bool Returns true if valid.
*/
bool isBcdMinSecValValid(uint8_t bcdVal) {
    bool isOk = true;
    uint8_t binVal = bcd_to_char(bcdVal);
    if (binVal > 59) {
        isOk = false;
    } else if ((bcdVal & 0xF) > 9) {
        isOk = false;
    } else if ((bcdVal >> 4) > 5) {
        isOk = false;
    }
    return isOk;
}

/**
* \brief Validate a 24 Hour BCD value for a legal range.
* 
* @param bcdVal The bcd value to check (0-23)
* 
* @return bool Returns true if valid.
*/
bool isBcdHour24Valid(uint8_t bcdVal) {
    bool isOk = true;
    uint8_t binVal = bcd_to_char(bcdVal);
    uint8_t tens = bcdVal >> 4;
    uint8_t ones = bcdVal & 0xF;
    if (binVal > 23) {
        isOk = false;
    } else if (tens > 2) {
        isOk = false;
    } else if ((tens < 2) && (ones > 9)) {
        isOk = false;
    } else if ((tens == 2) && (ones > 3)) {
        isOk = false;
    }
    return isOk;
}

/**
* \brief Utility routine to convert an hour, minute, and second 
*        time into its equivalent number of seconds.
* 
* @param hours    0-23
* @param minutes  0-59
* @param seconds  0-59
* 
* @return uint32_t Number of seconds 
*/
uint32_t timeInSeconds(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    uint32_t timeInSeconds = 0;
    uint32_t temp32;
    temp32 = hours;
    temp32 *= SECONDS_PER_HOUR;
    timeInSeconds = temp32;
    temp32 = minutes;
    temp32 *= SECONDS_PER_MINUTE;
    timeInSeconds += temp32;
    timeInSeconds += seconds;
    return timeInSeconds;
}

/**
* \brief Utility function to calculate the difference between 
*        two times given in hours, minutes and seconds.  The
*        result is given in seconds.
* 
* @param timeCompareP Pointer to structure containing the two 
*                     times including hours, minutes and
*                     seconds, and where to store the result
*                     (see the timeCompare_t definition).
*/
void calcTimeDiffInSeconds(timeCompare_t *timeCompareP) {
    int32_t timeInSecondsA = timeInSeconds(timeCompareP->hoursA, timeCompareP->minutesA, timeCompareP->secondsA);
    int32_t timeInSecondsB = timeInSeconds(timeCompareP->hoursB, timeCompareP->minutesB, timeCompareP->secondsB);
    int32_t timeDiffInSeconds = timeInSecondsA - timeInSecondsB;
    if (timeDiffInSeconds < 0) {
        timeDiffInSeconds += SECONDS_PER_DAY;
    }
    timeCompareP->timeDiffInSeconds = timeDiffInSeconds;
}

#if 0
static const uint8_t crc_table[] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3
};

uint8_t crc8(uint8_t *p, uint8_t len){
    uint16_t i;
    uint16_t crc = 0x0;

    while (len--) {
        i = (crc ^ *p++) & 0xFF;
        crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
    }

    return crc & 0xFF;
}
#endif

