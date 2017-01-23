/** 
 * @file i2c.c
 * \n Source File
 * \n Cascade MSP430 Firmware
 * 
 * \brief Support functions to write/read to/from the i2c bus 
 *        using the MSP430 USCI-B interface. 
 */

#include "cascade.h"

/***************************
 * Module Data Definitions
 **************************/

#define SDA_PIN BIT1  // UCB0SDA pin
#define SCL_PIN BIT2  // UCB0SCL pin

#define I2C_NFC_SLAVE_ADDRESS 0x55
#define I2C_TX_BUF_SIZE ((uint8_t)32)
#define I2C_RX_BUF_SIZE ((uint8_t)32)
#define SCL_CLOCK_DIV 0x12

typedef struct i2cData_s {
    int8_t byteCounter;
    uint8_t *rxBufP;
    uint8_t *txBufP;
    bool rxDone;
    bool txDone;
    bool gotNack;
    bool timeOut;
    sys_tick_t timestamp;
} i2cData_t;

/***************************
 * Module Data Declarations
 **************************/

i2cData_t i2cData;

static uint8_t i2cTxBuf[I2C_TX_BUF_SIZE];
static uint8_t i2cRxBuf[I2C_RX_BUF_SIZE];

/**************************
 * Module Prototypes
 **************************/

static void i2c_msp430RxInit(uint8_t slave_address, uint8_t prescale);
static void i2c_msp430TxInit(uint8_t slave_address, uint8_t prescale);
static int8_t i2c_receive(uint8_t *bufP, uint8_t byteCount);
static int8_t i2c_transmit(uint8_t *bufP, uint8_t byteCount);
static void uscib_rx_poll(void);
static void uscib_tx_poll(void);

/***************************
 * Module Public Functions 
 **************************/

/**
 * Performs a read operation on the I2C bus and retries it if unsuccessful.
 * @param buf Read buffer
 * @param len Length of data to read
 * @param slave I2C address of slave device
 * @param addr Register address
 * @return #ERROR_SUCCESS on success; #ERROR_I2C_FAILED otherwise
 */
error_t i2c_read(uint8_t *buf, unsigned int length, uint8_t slave, uint8_t addr) {

    int8_t status = 0;

    // Make sure interrupts are disabled
    UC0IE &= ~UCA0RXIE;
    UC0IE &= ~UCA0TXIE;

    // Setup to perform I2C Write
    i2c_msp430TxInit(slave, SCL_CLOCK_DIV);
    _delay_cycles(1000);

    // Perform I2C write of the address to read from slave
    i2cTxBuf[0] = addr;
    status = i2c_transmit(i2cTxBuf, 1);
    if (status == 0) {

        // Setup to perfrom I2c Read
        i2c_msp430RxInit(slave, SCL_CLOCK_DIV);
        _delay_cycles(1000);

        // Perform I2C Read
        status = i2c_receive(i2cRxBuf, length);
        if (status == 0) {
            // Copy data we read into user buffer
            memcpy(buf, i2cRxBuf, length);
        }
    }

    return status;
}

/**
 * Performs a write operation on the I2C bus and retries it if unsuccessful.
 * @param slave I2C address of slave device
 * @param addr Register address
 * @param data Data to write
 * @param length Length of data to write
 * @return #ERROR_SUCCESS on success; #ERROR_I2C_FAILED otherwise
 */
error_t i2c_write(uint8_t slave, uint8_t addr, const uint8_t *data, unsigned int length) {

    int8_t status = 0;

    // Make sure I2C interrupts are disabled
    UC0IE &= ~UCA0RXIE;
    UC0IE &= ~UCA0TXIE;

    // Setup for I2C write on MSP430
    i2c_msp430TxInit(slave, SCL_CLOCK_DIV);
    _delay_cycles(1000);

    // First byte is the register/address we want to write to on the slave device
    i2cTxBuf[0] = addr;
    // Add data from user to I2C buffer after the register/address byte
    memcpy(&i2cTxBuf[1], data, length);
    length += 1;

    // Perform I2C write
    status = i2c_transmit(i2cTxBuf, length);

    return status;
}

/**
* @brief Receive a buffer of data from the I2C slave
* 
* @param bufP  Buffer to store data received into
* @param byteCount  Number of expected bytes
* 
* @return error_t Returns 0 on success, -1 otherwise
*/
static int8_t i2c_receive(uint8_t *bufP, uint8_t byteCount) {
    bool error = false;
    sys_tick_t timestamp = GET_SYSTEM_TICK();

    i2cData.rxBufP = bufP;
    i2cData.rxDone = false;
    i2cData.gotNack = false;
    i2cData.timeOut = false;

    // Receiving only one byte is handled a bit differently.
    if (byteCount == 1) {
        // Because we are only receiving one byte, we need to setup the stop
        // condition immediately after the start condition is sent based on
        // the needs of the MSP430.
        i2cData.byteCounter = 0;

        __disable_interrupt();
        // I2C start condition
        UCB0CTL1 |= UCTXSTT;
        // Wait for start sent
        while (UCB0CTL1 & UCTXSTT);
        // I2C Stop Condition
        UCB0CTL1 |= UCTXSTP;
        __enable_interrupt();

    } else if (byteCount > 1) {
        i2cData.byteCounter = byteCount - 1;
        // Send I2C Start Condition
        UCB0CTL1 |= UCTXSTT;
    }

    // Wait for receive complete
    while (!error && !i2cData.rxDone) {
        // Use polling to drive the i2c receive process
        uscib_rx_poll();
        // Check for a timeout or NACK
        if (GET_ELAPSED_TIME_IN_SEC(timestamp) > 1) {
            i2cData.timeOut = true;
            error = true;
        }  else if (i2cData.gotNack) {
            error = true;
        }
    }

    _delay_cycles(1000);

    return error ? -1 : 0;

}

/**
* @brief transmit a buffer of data to the I2C slave device
* 
* @param txBufP  Buffer to transmit
* @param byteCount  Number of bytes to transmit
* 
* @return int8_t Returns 0 if success, -1 if failure
*/
static int8_t i2c_transmit(uint8_t *txBufP, uint8_t byteCount) {
    bool error = false;
    sys_tick_t timestamp = GET_SYSTEM_TICK();

    i2cData.txBufP = txBufP;
    i2cData.byteCounter = byteCount;

    i2cData.txDone = false;
    i2cData.gotNack = false;
    i2cData.timeOut = false;

    // I2C TX, start condition
    UCB0CTL1 |= UCTR + UCTXSTT;

    // Wait for transmit complete
    while (!error && !i2cData.txDone) {
        // Use polling to drive the i2c transmit process
        uscib_tx_poll();
        // Check for a timeout or NACK
        if (GET_ELAPSED_TIME_IN_SEC(timestamp) > 1) {
            i2cData.timeOut = true;
            error = true;
        }  else if (i2cData.gotNack) {
            error = true;
        }
    }

    _delay_cycles(1000);

    return error ? -1 : 0;
}

/**
* \brief Perform I2C low level rx processing.  We are using 
*        polling instead of an interrupt.  So do the things the
*        interrupt routine would normally do.
*/
static void uscib_rx_poll(void) {

    if (UCB0STAT & UCNACKIFG) {
        // send STOP if slave sends NACK
        UCB0CTL1 |= UCTXSTP;
        UCB0STAT &= ~UCNACKIFG;
        i2cData.gotNack = true;
    }

    if (IFG2 & UCB0RXIFG) {
        if (i2cData.byteCounter == 0) {
            // I2C stop condition
            UCB0CTL1 |= UCTXSTP;
            *i2cData.rxBufP = UCB0RXBUF;
            i2cData.rxBufP++;
            i2cData.rxDone = true;
        } else {
            *i2cData.rxBufP = UCB0RXBUF;
            i2cData.rxBufP++;
            i2cData.byteCounter--;
        }
    }
}

/**
* \brief Perform I2C low level tx processing.  We are using 
*        polling instead of an interrupt.  So do the things the
*        interrupt routine would normally do.
*/
static void uscib_tx_poll(void) {

    if (UCB0STAT & UCNACKIFG) {    // send STOP if slave sends NACK
        UCB0CTL1 |= UCTXSTP;
        UCB0STAT &= ~UCNACKIFG;
        i2cData.gotNack = true;
    }

    if (IFG2 & UCB0TXIFG) {
        if (i2cData.byteCounter == 0) {
            UCB0CTL1 |= UCTXSTP;   // I2C stop condition
            IFG2 &= ~UCB0TXIFG;    // Clear USCI_B0 TX int flag
            i2cData.txDone = true;
        } else {
            UCB0TXBUF = *i2cData.txBufP;
            i2cData.txBufP++;
            i2cData.byteCounter--;
        }
    }
}

#if 0
void i2c_test(void) {
    i2c_msp430TxInit(I2C_NFC_SLAVE_ADDRESS, SCL_CLOCK_DIV);
    i2cTxBuf[0] = 0x0;
    i2c_transmit(1);
    i2cData.txDone = false;
    while (!i2cData.txDone) {
        uscib_tx_poll();
    }
    i2c_msp430RxInit(I2C_NFC_SLAVE_ADDRESS, SCL_CLOCK_DIV);
    i2c_receive(16);
    i2cData.rxDone = false;
    while (!i2cData.rxDone) {
        uscib_rx_poll();
    }
}
#endif

//------------------------------------------------------------------------------
// void i2c_msp430RxInit(uint8_t slave_address,
//                              uint8_t prescale)
//
// This function initializes the USCI module for master-receive operation.
//
// IN:   uint8_t slave_address   =>  Slave Address
//       uint8_t prescale        =>  SCL clock adjustment
//-----------------------------------------------------------------------------
static void i2c_msp430RxInit(uint8_t slave_address, uint8_t prescale) {
    P3SEL |= SDA_PIN + SCL_PIN;            // Assign I2C pins to USCI_B0
    UCB0CTL1 = UCSWRST;                    // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;  // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;         // Use SMCLK, keep SW reset
    UCB0BR0 = prescale;                    // set prescaler
    UCB0BR1 = 0;
    UCB0I2CSA = slave_address;             // set slave address
    UCB0CTL1 &= ~UCSWRST;                  // Clear SW reset, resume operation
}

//------------------------------------------------------------------------------
// void i2c_msp430TxInit(uint8_t slave_address,
//                               uint8_t prescale)
//
// This function initializes the USCI module for master-transmit operation.
//
// IN:   uint8_t slave_address   =>  Slave Address
//       uint8_t prescale        =>  SCL clock adjustment
//------------------------------------------------------------------------------
static void i2c_msp430TxInit(uint8_t slave_address, uint8_t prescale) {
    P3SEL |= SDA_PIN + SCL_PIN;             // Assign I2C pins to USCI_B0
    UCB0CTL1 = UCSWRST;                     // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;   // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;          // Use SMCLK, keep SW reset
    UCB0BR0 = prescale;                     // set prescaler
    UCB0BR1 = 0;
    UCB0I2CSA = slave_address;              // Set slave address
    UCB0CTL1 &= ~UCSWRST;                   // Clear SW reset, resume operation
}

