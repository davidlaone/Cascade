/** 
 * @file modemLink.c
 * \n Source File
 * \n Cascade MSP430 Bootloader Firmware
 * 
 * \brief modem module responsible for bringing up the modem and 
 *        shutting it down.
 */

#include "cascade.h"

/***************************
 * Module Data Definitions
 **************************/

/**
 * \typedef modemPowerOnSeqState_t
 * \brief Define the different states to power on the modem.
 */
typedef enum modemPowerOnSeqState_e {
    MODEM_POWERUP_STATE_IDLE,
    MODEM_POWERUP_STATE_ALL_OFF,
    MODEM_POWERUP_STATE_DCDC,
    MODEM_POWERUP_STATE_LSVCC,
    MODEM_POWERUP_STATE_GSM_HIGH,
    MODEM_POWERUP_STATE_GSM_LOW,
    MODEM_POWERUP_STATE_INIT_WAIT,
    MODEM_POWERUP_STATE_READY,
} modemPowerOnSeqState_t;

/**
 * \typedef modemLinkData_t
 * \brief Module data structure.
 */
typedef struct modemLinkData_s {
    bool active;
    bool modemUp;
    sys_tick_t startTimestamp;
    modemPowerOnSeqState_t powerOnHwSeqState;
} modemLinkData_t;

/****************************
 * Module Data Declarations
 ***************************/

/**
 * \var mlData
 * \brief Declare the object that contains the module data.
 */
// static
modemLinkData_t mlData;

/********************* 
 * Module Prototypes
 *********************/

static bool modemPowerUpStateMachine(void);

/***************************
 * Module Public Functions
 **************************/

/**
* \brief Executive that manages sequencing the modem power up 
*        and power down.  Called on the 1 second tick rate to
*        handle bringing the modem up and down.
* \ingroup EXEC_ROUTINE
*/
void modemPower_exec(void) {
    if (mlData.active) {
        modemPowerUpStateMachine();
    }
}

/**
* \brief One time initialization for module.  Call one time 
*        after system starts or (optionally) wakes.
* \ingroup PUBLIC_API
*/
void modemPower_init(void) {
    memset(&mlData, 0, sizeof(modemLinkData_t));
}

/**
* \brief Start the modem power up sequence.  Will reset the 
*        state machine that manages the power up steps.
* \ingroup PUBLIC_API
*/
void modemPower_restart(void) {
    mlData.active = true;
    mlData.modemUp = false;
    mlData.powerOnHwSeqState = MODEM_POWERUP_STATE_ALL_OFF;
    mlData.startTimestamp = GET_SYSTEM_TICK();
    modemPowerUpStateMachine();
}

void modemPower_powerDownModem(void) {
    mlData.active = false;
    mlData.modemUp = false;
    mlData.powerOnHwSeqState = MODEM_POWERUP_STATE_IDLE;
    P1OUT &= ~GSM_DCDC;
    P1OUT &= ~LS_VCC;
}

bool modemPower_isModemOn(void) {
    return mlData.modemUp;
}

uint16_t modemPower_getModemUpTimeInSysTicks(void) {
    sys_tick_t onTime = GET_ELAPSED_SYS_TICKS(mlData.startTimestamp);
    uint16_t onTime16 = onTime;
    return onTime16;
}

/*************************
 * Module Private Functions
 ************************/

/**
* \brief State machine to sequence through the modem power on 
*        hardware steps. This is a timebased sequence to control
*        the hardware power on.
*/
static bool modemPowerUpStateMachine(void) {
    // Continue processing is used to determine whether
    // the do-while state processing loop should continue
    // for another iteration.
    bool continue_processing = false;
    volatile sys_tick_t onTime = GET_ELAPSED_SYS_TICKS(mlData.startTimestamp);
    switch (mlData.powerOnHwSeqState) {
    case MODEM_POWERUP_STATE_IDLE:
        break;
    case MODEM_POWERUP_STATE_ALL_OFF:
        P1OUT &= ~(GSM_DCDC|LS_VCC);
        if (onTime >= (1*SYS_TICKS_PER_SECOND)) {
            mlData.powerOnHwSeqState = MODEM_POWERUP_STATE_DCDC;
        }
        break;
    case MODEM_POWERUP_STATE_DCDC:
        if (onTime >= (2*SYS_TICKS_PER_SECOND)) {
            P1OUT |= GSM_DCDC;
            mlData.powerOnHwSeqState = MODEM_POWERUP_STATE_LSVCC;
        }
        break;
    case MODEM_POWERUP_STATE_LSVCC:
        if (onTime >= (3*SYS_TICKS_PER_SECOND)) {
            P1OUT |= LS_VCC;
            mlData.powerOnHwSeqState = MODEM_POWERUP_STATE_GSM_HIGH;
        }
        break;
    case MODEM_POWERUP_STATE_GSM_HIGH:
        if (onTime >= (4*SYS_TICKS_PER_SECOND)) {
            P1OUT |= GSM_EN;
            mlData.powerOnHwSeqState = MODEM_POWERUP_STATE_GSM_LOW;
        }
        break;
    case MODEM_POWERUP_STATE_GSM_LOW:
        if (onTime >= (10*SYS_TICKS_PER_SECOND)) {
            P1OUT &= ~GSM_EN;
            mlData.powerOnHwSeqState = MODEM_POWERUP_STATE_INIT_WAIT;
        }
        break;
    case MODEM_POWERUP_STATE_INIT_WAIT:
        if (onTime >= (15*SYS_TICKS_PER_SECOND)) {
            mlData.powerOnHwSeqState = MODEM_POWERUP_STATE_READY;
            mlData.modemUp = true;
        }
        break;
    case MODEM_POWERUP_STATE_READY:
        break;
    }
    return continue_processing;
}

