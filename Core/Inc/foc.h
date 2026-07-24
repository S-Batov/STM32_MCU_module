/**
 * @file foc.h
 * @brief Header file with function definitions and variables
 * needed to run FOC
 */

#ifndef _FOC_H_
#define _FOC_H_

#include "main.h"
#include <stdint.h>
#include "RampGen.h"

//#define ADC2_CH_NUM 5
#define ADC3_CH_NUM 7

/// FOC status enumeration
typedef enum {
	FOC_ALL_GOOD = 0,
	FOC_INIT_FAULT = 1,
	FOC_STATUS_FAULT = 2,
} FOC_Status_e;


//uint16_t ADC2_DMA_buff[ADC2_CH_NUM];
extern uint16_t ADC3_DMA_buff[ADC3_CH_NUM];

#define I_U_HI  0 /* Rank 1 - CH2  */
#define I_V_HI  1 /* Rank 2 - CH15 */
#define I_U_LO  2 /* Rank 3 - CH14 */
#define I_V_LO  3 /* Rank 4 - CH16 */
#define I_DC_HI 4 /* Rank 5 - CH4  */
#define I_DC_LO 5 /* Rank 6 - CH6  */
#define V_DC    6 /* Rank 7 - CH3  */

typedef struct
{
    float Kp;         /* Proportional gain for the PI controller      */
    float Ki;         /* Integral gain for the PI controller          */
    float Ui;         /* Integrator start value for the PI controller */

    float refValue;   /* Reference input value   */
    float fbackValue; /* Feedback input value    */
    float ffwdValue;  /* Feedforward input value */
    float outMin;     /* Minimum output value allowed for the PI controller */
    float outMax;     /* Maximum output value allowed for the PI controller */
} PI_Obj;

typedef struct
{
	  /* --- Motor parameters / constants ------------------------------------------*/
    float Ls_d;          /* D-axis inductance */
    float Ls_q;          /* Q-axis inductance */
    float Rs;            /* Stator resistance */
    float Phi_e;         /* Permanent magnet flux linkage */
                         /* Motor parameter, used in decoupling function */
    uint16_t pole_pairs; /* Motor parameter, used to convert mechanical to electrical angle */

    float I_uvw_A[3]; /* Measured phase currents */
    float V_uvw_V[3]; /* Phase voltages */
                      /* Unused */

    float I_scale; /* Scaling factor for current measurements */
    float V_scale; /* Scaling factor for DC link voltage measurement */

    float dcBus_V;           /* DC link voltage */
    float oneOverDcBus_invV; /* inverse of DC link voltage */

    float I_ab_A[2]; /* Currents in the alpha-beta reference frame */
    float I_dq_A[2]; /* Currents in the D-Q reference frame        */

    uint32_t encoder_count; /* Rising edge count of the ABZ encoder */
    float theta_e; /* Rotor ELECTRICAL angle in PU */
    float omega_e; /* Rotor ELECTRICAL speed in rad/s */

    float Sine;   /* Sine of the theta value   */
    float Cosine; /* Cosine of the theta value */

    float Vff_dq_V[2];  /* PI_dq FF value after decoupling                */
    float Vout_dq_V[2]; /* Corrective voltages Vd and Vq after PI control */
    float Vout_ab_V[2]; /* Valpha and Vbeta, after inverse park transform */

    PI_Obj pi_id; /* Id PI regulator */
    PI_Obj pi_iq; /* Iq PI regulator */

    float vqLimit;				 /* Unused */
    float modulationLimitSquare; /* Unused */
} Motor_t;

/**
 * @brief Initialize the motor struct with default parameters
 *
 * @param motor [in/out] - Pointer to motor struct to be initialized
 */
void FOC_motor_init(Motor_t *motor);

/**
 * @brief Start sampling of ADC3 (phase currents, DC link) on
 * TIM8 TRGO event and transfer them to ADC3_DMA_buff
 *
 * @note This function should be called before starting the PWM timer
 */
HAL_StatusTypeDef FOC_start_ADC_DMA(void);

FOC_Status_e FOC_encoder_init(Motor_t *motor);

void FOC_run(Motor_t *motor, RampGen *rg);

#endif /* _FOC_H_ */
