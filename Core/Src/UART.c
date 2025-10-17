/**
 * @file UART.c
 * @brief UART communication functions.
 *
 * Implements blocking UART transmit/receive routines for communicating over the UART1 peripheral.
 */

#include "UART.h"

// Use the global handle from main.c
extern UART_HandleTypeDef huart1;

HAL_StatusTypeDef UART_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_UART_Transmit(&huart1, pData, Size, Timeout);
}

HAL_StatusTypeDef UART_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_UART_Receive(&huart1, pData, Size, Timeout);
}
