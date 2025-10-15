//#############################################################################
// FILE    : UCC5870.h
// TITLE   : Header file for UCC5870 interface modules
// Version : 1.0
//
//  Group         : C2000
//  Target Family : F2837x
//  Created on    : Jun 3, 2020
//  Author        : Ramesh Ramamoorthy
//#############################################################################
// $TI Release: C2000 FCL SFRA $
// $Release Date: 05/2020 $
// $Copyright: Copyright (C) 2013-2018 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

#ifndef _UCC5870_H_
#define _UCC5870_H_

#include "stm32g4xx_hal.h"
#include "ucc5870_regs.h"

#define DRIVER_NUM 6

//
// enumerated variables
//
typedef enum {
	ALL_GOOD = 0,
	INIT_FAULT = 1,
	STATUS_FAULT = 2,
	PRI_RDY_FAULT = 3,
	SEC_RDY_FAULT = 4
} UCC5870_Status_e;

/*****************************************************************************/
// function prototypes
/*****************************************************************************/

void Init_UCC5870_Regs(void);
uint16_t diagnose_UCC5870(uint16_t i);
void clearFaultsUCC5870(void);
UCC5870_Status_e inverterDiagnostics();
UCC5870_Status_e Init_UCC5870();
void printInverterStatus();

// local prototypes for use within ucc5870.c
uint16_t sendCmdUCC5870(uint16_t tx_data);
uint16_t readRegUCC5870(uint16_t CA, uint16_t RA);
void writeRegUCC5870(uint16_t CA, uint16_t RA, uint16_t data);
uint16_t writeVerifyRegUCC5870(uint16_t CA, uint16_t RA, uint16_t data);
uint16_t writeVerify_UCC5870(uint16_t CA);


//=============================================================================
#endif /* _UCC5870_H_ */
