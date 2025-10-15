#ifndef _SPI_H_
#define _SPI_H_

#include "main.h"

// Use the global handle from main.c
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;


// =============================================================================
// SPI1 interface functions for communication with UCC5870
// =============================================================================

// Transmit + Receive (full duplex)
HAL_StatusTypeDef SPI_TransmitReceive_Driver(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);


// =============================================================================
// SPI4 interface functions for communication with an SSI encoder
// =============================================================================

// Transmit + Receive (full duplex)
HAL_StatusTypeDef SPI_TransmitReceive_Encoder(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

#endif /* _SPI_H_ */
