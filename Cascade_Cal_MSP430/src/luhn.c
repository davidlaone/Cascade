/** 
 * @file luhn.c
 * \n Source File
 * \n Cascade MSP430 Firmware
 * 
 * \brief Support for calculating a Luhn digit.
 *  
 * \note This module will not link correctly if the optimizer is
 *       turned off.  Apparently with the optimizer off,
 *       argument placement (i.e. the calling conventions) for
 *       the 64 bit support functions for multiply and divide
 *       are affected. The causes the linker to get confused for
 *       some reason. Hence, that is why this function is
 *       isolated to this module and will always be compiled
 *       with the optimizer on.
 */
#include "cascade.h"

/**
* @brief Appends a Luhn check digit to the ID specified.
* 
* @param val The value to encode
* 
* @return uint64_t Encoded value
*/
uint64_t luhn(uint64_t val) {
    uint64_t num = val;
    unsigned int sum = 0, d;
    unsigned char odd;

    for (odd = 1; num > 0; num /= 10, odd = !odd) {
        d = (int)(num % 10);
        if (odd) {
            d *= 2;
            if (d > 9) d -= 9;
        }
        sum += d;
    }

    return val * 10ULL + (uint64_t)((10 - sum % 10) % 10);
}

/**
* @brief Convert the uin64_t luhn value to ASCII.  Assumes its 
*        in the form of an IMEI value encoded with the luhn
*        digit.
* 
* @param val The value to convert to ASCII
* @param bufP The buffer to store the ASCII values
* 
* @return uint8_t The number of ASCII characters stored in the 
*         buffer.
*/
uint8_t imeiToAscii(uint64_t val, uint8_t *bufP) {
    unsigned int d;
    uint8_t tempBuf[32];
    int8_t i;
    uint8_t resultLength = 0;
    for (i = 0; (val > 0) && (i < 32); val /= 10, i++) {
        d = (int)(val % 10);
        tempBuf[i] = 0x30 + d;
    }
    resultLength = i;
    for (i -= 1; i >= 0; i--) {
        *bufP++ = tempBuf[i];
    }
    return resultLength;
}

/**
* \brief Convert an array of eight bytes into a 64 unsigned 
*        integer.
* 
* @return uint64_t 
*/
uint64_t imeiArrayToInt (uint8_t *bufP) {
    uint64_t imeiVal;
    uint64_t byteVal;

    byteVal = *bufP++;
    imeiVal = byteVal;
    imeiVal *= 256ULL;

    byteVal = *bufP++;
    imeiVal += byteVal;
    imeiVal *= 256ULL;

    byteVal = *bufP++;
    imeiVal += byteVal;
    imeiVal *= 256ULL;

    byteVal = *bufP++;
    imeiVal += byteVal;
    imeiVal *= 256ULL;

    byteVal = *bufP++;
    imeiVal += byteVal;
    imeiVal *= 256ULL;

    byteVal = *bufP++;
    imeiVal += byteVal;
    imeiVal *= 256ULL;

    byteVal = *bufP++;
    imeiVal += byteVal;
    imeiVal *= 256ULL;

    byteVal = *bufP++;
    imeiVal += byteVal;

    return imeiVal;
}
