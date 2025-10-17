/**
 * @file UART.h
 * @brief UART peripheral interface header
 *
 * This header provides transmit and receive functions for UART, allowing other
 * modules to access UART functionality without directly including "main.h".
 *
 * Include this file in any source that requires UART communication.
 *
 * @note This file does not configure or initialize the UART peripheral -
 * initialization is performed in "main.c" using CubeMX-generated code.
 */


#ifndef _UART_H_
#define _UART_H_

#include "main.h"

/**
 * @brief Transmit data over UART1
 *
 * @param[in] pData   - Pointer to transmit data buffer
 * @param[in] Size    - Number of frames to transmit
 * @param[in] Timeout - Timeout duration in milliseconds
 *
 * @return HAL_StatusTypeDef
 *         - HAL_OK on success
 *         - HAL_ERROR on UART error
 *         - HAL_TIMEOUT if operation times out
 *
 * @note The UART1 peripheral must be initialized before calling this function
 * @warning This function is blocking; use it only when timing requirements allow it.
 */
HAL_StatusTypeDef UART_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
 * @brief Receive data over UART1
 *
 * @param[in] pData   - Pointer to receive data buffer
 * @param[in] Size    - Number of frames to receive
 * @param[in] Timeout - Timeout duration in milliseconds
 *
 * @return HAL_StatusTypeDef
 *         - HAL_OK on success
 *         - HAL_ERROR on SPI error
 *         - HAL_TIMEOUT if operation times out
 *
 * @note The UART1 peripheral must be initialized before calling this function
 * @warning This function is blocking; use it only when timing requirements allow it.
 */
HAL_StatusTypeDef UART_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout);



#endif /*_UART_H_ */
