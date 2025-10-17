/**
 * @file SPI.h
 * @brief SPI peripheral interface header
 *
 * This header provides transmit and receive functions for SPI, allowing other
 * modules to access SPI functionality without directly including "main.h".
 *
 * Include this file in any source that requires SPI communication.
 *
 * @note This file does not configure or initialize the SPI peripheral -
 * initialization is performed in "main.c" using CubeMX-generated code.
 */

#ifndef _SPI_H_
#define _SPI_H_

#include "main.h"

/**
 * @brief Transmit and receive data over SPI1 interface used for gate driver communication
 *
 * This function performs a full-duplex SPI transaction using the global SPI1 handle.
 *
 * @param[in]  pTxData - Pointer to transmit data buffer.
 * @param[out] pRxData - Pointer to receive data buffer.
 * @param[in]  Size    - Number of frames to transmit/receive
 * @param[in]  Timeout - Timeout duration in milliseconds
 *
 * @return HAL_StatusTypeDef
 *         - HAL_OK on success
 *         - HAL_ERROR on SPI error
 *         - HAL_TIMEOUT if operation times out
 *
 * @note The SPI1 peripheral must be initialized before calling this function
 * @warning This function is blocking; use it only when timing requirements allow it.
 */
HAL_StatusTypeDef SPI_TransmitReceive_Driver(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);


/**
 * @brief Transmit and receive data over SPI4 interface used for communication with SSI encoders
 *
 * This function performs a full-duplex SPI transaction using the global SPI4 handle.
 *
 * @param[in]  pTxData - Pointer to transmit data buffer.
 * @param[out] pRxData - Pointer to receive data buffer.
 * @param[in]  Size    - Number of frames to transmit/receive
 * @param[in]  Timeout - Timeout duration in milliseconds
 *
 * @return HAL_StatusTypeDef
 *         - HAL_OK on success
 *         - HAL_ERROR on SPI error
 *         - HAL_TIMEOUT if operation times out
 *
 * @note The SPI4 peripheral must be initialized before calling this function
 * @warning This function is blocking; use it only when timing requirements allow it.
 */
HAL_StatusTypeDef SPI_TransmitReceive_Encoder(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

#endif /* _SPI_H_ */
