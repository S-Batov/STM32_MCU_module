#include "SPI.h"

HAL_StatusTypeDef SPI_TransmitReceive_Driver(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
    return HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, Size, Timeout);
}

HAL_StatusTypeDef SPI_TransmitReceive_Encoder(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
    return HAL_SPI_TransmitReceive(&hspi4, pTxData, pRxData, Size, Timeout);
}
