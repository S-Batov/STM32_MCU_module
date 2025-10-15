#include "PWM_timer.h"
#include "stm32g474xx.h"   // Brings in IRQn_Type definitions
#include "stm32g4xx_hal.h"
#include <math.h>

#define SINE_TABLE_SIZE 256
#define PWM_MAX_DUTY    3999    // ARR value for 16 kHz PWM (0..3999)

static uint16_t sine_dma_buffer[SINE_TABLE_SIZE * 3];

/**
 * @brief Main output disable
 */
void PWM_timer_MOE_disable()
{
	TIM8->BDTR &= ~TIM_BDTR_MOE;
}

/**
 * @brief Main output enable
 */
void PWM_timer_MOE_enable()
{
	TIM8->BDTR |= TIM_BDTR_MOE;
}

/**
 * @brief force update event
 *
 * Force an update event in order to update the ARR,
 * CCR and BDTR registers with initial values. Required to
 * avoid causing an STP fault on first cycle (due to no dead time)
 */
void PWM_timer_force_update()
{
    TIM8->EGR = TIM_EGR_UG;
}

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

/**
 * @brief start DMA transfer for PWM timer
 *
 * @return HAL_OK if DMA successfully started
 *         HAL_ERROR otherwise
 */
HAL_StatusTypeDef PWM_timer_DMA_start()
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

/**
 * @brief start PWM timer
 *
 * Steps to start the timer:
 *  1) Disable MOE
 *  2) Start PWM outputs
 *  3) Enable interrupts
 *  4) Enable MOE
 *
 *	@return HAL_OK if all 3 channels of PWM timer successfully started,
 *          HAL_ERROR otherwise
 */
HAL_StatusTypeDef PWM_timer_start()
{
	// =============================================================================
	// 1) Diable MOE
	PWM_timer_MOE_disable();

	// =============================================================================
	// 2) Start PWM outputs
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

	// =============================================================================
	// 3) Peripheral level enable for the break and update interrupts
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);

    PWM_timer_force_update(); 	// Force a TIM8 update event to load correct
    							// dead time value for the first cycle

	// =============================================================================
	// 4) Enable MOE
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



