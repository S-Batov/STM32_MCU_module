#ifndef _PWM_timer_H_
#define _PWM_timer_H_

#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim8;

HAL_StatusTypeDef PWM_timer_DMA_start();

HAL_StatusTypeDef PWM_timer_start();

#endif // _PWM_timer_H_
