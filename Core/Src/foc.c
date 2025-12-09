/**
 * @file foc.c
 * @brief FOC functions to start and run FOC
 */

#include "foc.h"

extern ADC_HandleTypeDef hadc3;

uint16_t ADC3_DMA_buff[ADC3_CH_NUM];

void FOC_start_ADC_DMA(void)
{
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3_DMA_buff, ADC3_CH_NUM);
}
