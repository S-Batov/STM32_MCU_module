/**
 * @file PWM_timer.c
 * @brief PWM_timer functions needed to start the timer and DMA transfer
 */

#include "PWM_timer.h"
#include "stm32g474xx.h"   // Brings in IRQn_Type definitions
#include "stm32g4xx_hal.h"
#include <math.h>

#define SINE_TABLE_SIZE 256
#define PWM_MAX_DUTY    3999    // ARR value for 16 kHz PWM (0..3999)

static uint16_t sine_dma_buffer[SINE_TABLE_SIZE * 3];

extern TIM_HandleTypeDef htim8;

/**
 * @brief Main output disable
 *
 * Disables the PWM_timer output to MCU pins by setting the BDTR register bit MOE to 0.
 *
 * As per RM0440 chapter 29.6.20:
 * - 0: OC and OCN outputs are disabled or forced to idle state depending on the OSSI bit.
 * - 1: OC and OCN outputs are enabled if their respective enable bits are set
 *
 * @note This doesn't stop the timer or PWM generation within the MCU, only disables the generated
 * PWM to be output to the MCU pins. The timer will keep running and generating update/break interrupts.
 */
void PWM_timer_MOE_disable(void)
{
	TIM8->BDTR &= ~TIM_BDTR_MOE;
}

/**
 * @brief Main output enable
 *
 * Enables the PWM_timer output to MCU pins by setting the BDTR register bit MOE to 1.
 *
 * As per RM0440 chapter 29.6.20:
 * - 0: OC and OCN outputs are disabled or forced to idle state depending on the OSSI bit.
 * - 1: OC and OCN outputs are enabled if their respective enable bits are set
 */
void PWM_timer_MOE_enable(void)
{
	TIM8->BDTR |= TIM_BDTR_MOE;
}

/**
 * @brief Force update event
 *
 * Forces an update event by setting the TIM8 EGR register bit UG to 1,
 * in order to update the ARR, CCR and BDTR registers with initial values.
 *
 * As per RM0440 chapter 29.6.6:
 * This bit can be set by software, it is automatically cleared by hardware.
 * - 0: No action
 * - 1: Reinitialize the counter and generates an update of the registers
 *
 * @warning Usage of this function is required before setting MOE to avoid
 * causing a STP fault on first cycle (due to no dead time)
 */
void PWM_timer_force_update(void)
{
    TIM8->EGR = TIM_EGR_UG;
}

/**
 * @brief Initialize the array for sine-waves
 *
 * Initialize values in sine_dma_buffer to create three sine-waves phase shifted by 120 deg
 */
void Generate3PhaseSineTable(void)
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++)
    {
        float angle = (2.0f * M_PI * i) / SINE_TABLE_SIZE;

        float s1 = sinf(angle);
        float s2 = sinf(angle - 2.0f * M_PI / 3.0f);
        float s3 = sinf(angle - 4.0f * M_PI / 3.0f);

        sine_dma_buffer[i*3 + 0] = (uint16_t)((s1 * 0.5f + 0.5f) * PWM_MAX_DUTY);
        sine_dma_buffer[i*3 + 1] = (uint16_t)((s2 * 0.5f + 0.5f) * PWM_MAX_DUTY);
        sine_dma_buffer[i*3 + 2] = (uint16_t)((s3 * 0.5f + 0.5f) * PWM_MAX_DUTY);
    }
}

HAL_StatusTypeDef PWM_timer_DMA_start(void)
{
	HAL_StatusTypeDef status = HAL_ERROR;

    // Generate LUTs once
    Generate3PhaseSineTable();

    status = HAL_TIM_DMABurst_MultiWriteStart(&htim8,
    							TIM_DMABASE_CCR1,
								TIM_DMA_UPDATE,
                                (uint32_t*)sine_dma_buffer,
								TIM_DMABURSTLENGTH_3TRANSFERS,
								SINE_TABLE_SIZE * 3);
    if(status != HAL_OK)
    	return HAL_ERROR;

    PWM_timer_force_update(); 	// Force a TIM8 update event to load correct
    							// dead time value for the first cycle

    return HAL_OK;
}

HAL_StatusTypeDef PWM_timer_start(void)
{
	/* --- 1. Disable the Main Output Enable (MOE) to safely configure outputs ---*/
	PWM_timer_MOE_disable();

	/* --- 2. Starts TIM8 base counter and all PWM channels ----------------------*/
	if(HAL_TIM_Base_Start(&htim8) != HAL_OK)
		goto PWM_start_error;

	if(HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK)
		goto PWM_start_error;

	if(HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK)
		goto PWM_start_error;

	if(HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK)
		goto PWM_start_error;

	if(HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1) != HAL_OK)
		goto PWM_start_error;

	if(HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2) != HAL_OK)
		goto PWM_start_error;

	if(HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3) != HAL_OK)
		goto PWM_start_error;

	/* --- 3. Enable update and break interrupts for the timer -------------------*/
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);

	/* --- 4. Force an update event to load dead time for the first cycle --------*/
    PWM_timer_force_update(); 	// Force a TIM8 update event to load correct
    							// dead time value for the first cycle

	/* --- 5. Enable the Main Output Enable (MOE) to start PWM on output pins ----*/
    PWM_timer_MOE_enable();

	return HAL_OK;

PWM_start_error:
	PWM_timer_MOE_disable();
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
	return HAL_ERROR;
}
