/**
 * @file foc.h
 * @brief Header file with function definitions and variables
 * needed to run FOC
 */

#ifndef _FOC_H_
#define _FOC_H_

#include "main.h"
#include <stdint.h>

//#define ADC2_CH_NUM 5
#define ADC3_CH_NUM 7

//uint16_t ADC2_DMA_buff[ADC2_CH_NUM];
extern uint16_t ADC3_DMA_buff[ADC3_CH_NUM];

#define I_U_HI  0 /* Rank 1 - CH2  */
#define I_V_HI  1 /* Rank 2 - CH15 */
#define I_U_LO  2 /* Rank 3 - CH14 */
#define I_V_LO  3 /* Rank 4 - CH16 */
#define I_DC_HI 4 /* Rank 5 - CH4  */
#define I_DC_LO 5 /* Rank 6 - CH6  */
#define V_DC    6 /* Rank 7 - CH3  */

//typedef struct _Motor_t_
//{
//    float32_t Ls_d;  /* D-axis inductance */
//    float32_t Ls_q;  /* Q-axis inductance */
//    float32_t Rs;    /* Stator resistance */
//    float32_t Phi_e; /* Permanent magnet flux linkage */
//    				 /* Motor parameter, used in decoupling function */
//
//    float32_t I_abc_A[3]; /* Phase currents */
//    float32_t V_abc_V[3]; /* Phase voltages */
//
//    float32_t I_scale;
//    float32_t V_scale;
//
//    float32_t dcBus_V;           /* DC link voltage */
//    float32_t oneOverDcBus_invV; /* inverse of DC link voltage */
//
//    float32_t I_ab_A[2]; /* Currents in the alpha-beta reference frame */
//    float32_t I_dq_A[2]; /* Currents in the D-Q reference frame */
//
//    float32_t theta_e; /* Rotor angle */
//    float32_t omega_e; /* Rotor speed */
//
//    float32_t Sine;
//    float32_t Cosine;
//
//    float32_t Vff_dq_V[2];          //!< PI_dq FF value for decoupling
//    float32_t Vout_dq_V[2];         //!< the current values
//    float32_t Vout_ab_V[2];         //!< the current values
//
//    PI_Obj pi_id;
//    PI_Obj pi_iq;
//
//    float32_t vqLimit;
//    float32_t modulationLimitSquare;
//} Motor_t;


/**
 * @brief Start sampling of ADC3 (phase currents, DC link) on
 * TIM8 TRGO event and transfer them to ADC3_DMA_buff
 *
 * @note This function should be called before starting the PWM timer
 */
void FOC_start_ADC_DMA(void);
/**
 * @brief Run the FOC algorithm
 *
 * @note This function should be called when TIM8_UP interrupt is generated
 *
 * TIM8 generates a TIM8_UP interrupt when PWM reaches 0 or ARR. At that point
 * TIM8 generates a TRGO event, which is used to start ADC sampling of phase currents
 * and DC link voltage and current.
 *
 * TODO: Use the TRGO event to obtain the rotor angle
 *
 * The algorithm steps are as follows:
 *  1. Obtain U, V, W and DC values and rotor angle
 *  2. Generate Iq and Id reference values (from torque request)
 *  3. Using Clarke-Parke transforms transform U, V, W currents to Iq and Id
 *  4. Calculate the Iq and Id error between reference and measured values
 *  5. Generate correction voltage Vq and Vd
 *  6. Generate the feed-forward correction voltage (decoupling, losses, back EMF...)
 *  7. Using inverse Clarke-Parke transforms transform the correction voltage to U, V, W voltages
 *  8. Compare the correction voltages to DC link voltage and generate PWM duty cycle for U, V, W
 */
void FOC_run(void);

#endif /* _FOC_H_ */
