/**
 * @file foc.h
 * @brief Header file with function definitions and variables
 * needed to run FOC
 */

#ifndef _FOC_H_
#define _FOC_H_

#include "main.h"

//#define ADC2_CH_NUM 5
#define ADC3_CH_NUM 7

//uint16_t ADC2_DMA_buff[ADC2_CH_NUM];
extern uint16_t ADC3_DMA_buff[ADC3_CH_NUM];

#define I_U_HI  0 /* Rank 1 - CH2  */
#define I_v_HI  1 /* Rank 2 - CH15 */
#define I_U_LO  2 /* Rank 3 - CH14 */
#define I_V_LO  3 /* Rank 4 - CH16 */
#define I_DC_HI 4 /* Rank 5 - CH4  */
#define I_DC_LO 5 /* Rank 6 - CH6  */
#define V_DC    6 /* Rank 7 - CH3  */

/**
 * @brief Start sampling of ADC3 (phase currents, DC link) on
 * TIM8 TRGO event and transfer them to ADC3_DMA_buff
 *
 * @note This function should be called before starting the PWM timer
 */
void FOC_start_ADC_DMA(void);

#endif /* _FOC_H_ */
