/**
 * @file SPI.c
 * @brief SPI communication functions for gate driver and SSI encoder interface.
 *
 * Implements blocking SPI transmit/receive routines for communicating
 * with gate driver ICs over the SPI1 peripheral, and SSI encoders on SPI4 interface.
 */

#include "SPI.h"

/// SPI1: used for gate driver communication
extern SPI_HandleTypeDef hspi1;

/// SPI4: used for SSI encoder communication
extern SPI_HandleTypeDef hspi4;

HAL_StatusTypeDef SPI_TransmitReceive_Driver(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
    return HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, Size, Timeout);
}

HAL_StatusTypeDef SPI_TransmitReceive_Encoder(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
    return HAL_SPI_TransmitReceive(&hspi4, pTxData, pRxData, Size, Timeout);
}
