#ifndef _UART_H_
#define _UART_H_

#include "main.h"

// Use the global handle from main.c
extern UART_HandleTypeDef huart1;

/**
  * @brief  Send data over LPUART1.
  * @param  pData: Pointer to buffer to send
  * @param  Size: Number of bytes
  * @param  Timeout: Timeout in ms
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
  * @brief  Receive data over LPUART1.
  * @param  pData: Pointer to buffer to receive into
  * @param  Size: Number of bytes
  * @param  Timeout: Timeout in ms
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout);



#endif /*_UART_H_ */
