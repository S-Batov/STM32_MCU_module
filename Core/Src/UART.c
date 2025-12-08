/**
 * @file UART.c
 * @brief UART communication functions.
 *
 * Implements blocking UART transmit/receive routines for communicating over the UART1 peripheral.
 */

#include "UART.h"

// Use the global handle from main.c
extern UART_HandleTypeDef huart1;

HAL_StatusTypeDef UART_Transmit(uint8_t *pData, uint16_t Size)
{
    return HAL_UART_Transmit_DMA(&huart1, pData, Size);
}

HAL_StatusTypeDef UART_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_UART_Receive(&huart1, pData, Size, Timeout);
}

uint8_t UART_is_ready()
{
	return HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY;
}
