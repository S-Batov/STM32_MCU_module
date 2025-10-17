/**
 * @file UCC5870.h
 * @brief Header file for UCC5870 interface modules
 *
 * Defines the public functions needed to interact with the gate drivers such as
 * writing to registers and reading from registers. Also defines functions to
 * diagnose faults and transmit the gate driver status register data to UART.
 */

#ifndef _UCC5870_H_
#define _UCC5870_H_

#include "stm32g4xx_hal.h"
#include "ucc5870_regs.h"

#define DRIVER_NUM 6

/// UCC5870 status enumeration
typedef enum {
	ALL_GOOD = 0,
	INIT_FAULT = 1,
	STATUS_FAULT = 2,
	PRI_RDY_FAULT = 3,
	SEC_RDY_FAULT = 4
} UCC5870_Status_e;

/* --- Function prototypes ---------------------------------------------------*/

/**
 * @brief Initialize local UCC5870 registers
 *
 * Initialize data to local UCC5870 registers. This is done by initializing the configuration
 * parameters of one UCC5870, which is then copied to all of the drivers.
 *
 * @note This function only initializes the values of local registers, it should be followed by
 * Init_UCC5870 for the configuration to be written to the actual gate driver ICs
 */
void Init_UCC5870_Regs(void);

/**
 * @brief Run diagnostics for all gate driver ICs
 *
 * Read status registers of all gate driver ICs and report if a fault was found.
 *
 * @return UCC5870_Status_e
 *         - ALL_GOOD if no faults were found on any of the gate drivers
 *         - STATUS_FAULT if a fault was found on any of the gate drivers
 *         - PRI_RDY_FAULT if the primary read flag is not set
 *         - SEC_RDY_FAULT if the secondary ready flag is not set
 */
UCC5870_Status_e inverterDiagnostics(void);

/**
 * @brief Initialize gate driver IC configuration registers and enable them
 *
 * Initializes the gate driver ICs by first assigning them unique addresses and enabling the ICs.
 * The configuration data from the local UCC5870 registers is transmitted via SPI. Configuration
 * data is then read back from the drivers in order to confirm the configuration was successful.
 * Finally, the the ICs' STATUS registers are checked to ensure no faults are present.
 *
 * @return UCC5870_Status_e
 *         - ALL_GOOD if configuration of all ICs' was successful
 *         - INIT_FAULT if data read from the IC doesn't match the data written to it OR faults are
 *         present after initialization
 */
UCC5870_Status_e Init_UCC5870(void);

/**
 * @brief Print values of status registers for all gate driver ICs
 *
 * Poll latest status register data from all gate driver ICs and transmit that data
 * to UART1
 *
 * @warning This function is blocking; use it only when timing requirements allow it.
 */
void printInverterStatus(void);

#endif /* _UCC5870_H_ */
