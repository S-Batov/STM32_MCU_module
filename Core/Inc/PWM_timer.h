/**
 * @file PWM_timer.h
 * @brief PWM timer peripheral interface header
 *
 * This header provides functions for interfacing with the PWM timer, such as
 * functions to start the timer and start the DMA transfer, allowing other
 * modules to access the PWM timer functionality without directly including "main.h"
 *
 * @note This file does not configure or initialize the TIM8 peripheral -
 * initialization is performed in "main.c" using CubeMX-generated code.
 */

#ifndef _PWM_timer_H_
#define _PWM_timer_H_

#include "stm32g4xx_hal.h"

/**
 * @brief Start DMA transfer for PWM timer
 *
 * Computes the PWM values needed to generate 3 sine-waves phase shifted by 120 deg.
 * Starts the DMA transfer from the array with computed values and PWM timer CCR register.
 * This produces 3 sine-waves shifted by 120 deg on the PWM timer output pins, without
 * any software overhead.
 *
 * @return HAL_StatusTypeDef
 *         - HAL_OK: DMA started successfully
 *         - HAL_ERROR: Failure in starting DMA channels
 *
 * @warning This function should be called before starting the PWM output to avoid static values
 * in the CCR register. Any reconfiguration of the DMA channels should be done with MOE disabled
 * to prevent glitches or incorrect output during the transition.
 */
HAL_StatusTypeDef PWM_timer_DMA_start(void);

/**
 * @brief Starts PWM outputs on TIM8 with complementary channels.
 *
 * This function performs the following steps:
 *  1. Disables the Main Output Enable (MOE) to safely configure outputs.
 *  2. Starts TIM8 base counter and all PWM channels (TIM_CHANNEL_1, 2, 3)
 *     including complementary outputs (N channels).
 *  3. Enables update and break interrupts for the timer.
 *  4. Forces an update event to correctly load dead time for the first cycle.
 *  5. Re-enables MOE to allow PWM outputs to drive the power stage.
 *
 * If any HAL start function fails, all channels are stopped and MOE is disabled,
 * returning HAL_ERROR.
 *
 * @return HAL_StatusTypeDef
 *         - HAL_OK: PWM started successfully
 *         - HAL_ERROR: Failure in starting timer or PWM channels
 */
HAL_StatusTypeDef PWM_timer_start(void);

#endif // _PWM_timer_H_
