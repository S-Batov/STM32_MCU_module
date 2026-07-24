/**
 * @file foc.c
 * @brief FOC functions to start and run FOC
 */

#include "foc.h"
#include "PWM_timer.h"
#include "SPI.h"
#include "CORDIC.h"
#include "GPIO_status.h"

#define PI_CONST         3.14159265359f
#define TWO_PI           6.28318530718
#define ONE_OVER_SQRT3   0.57735026918963 /* 1/sqrt(3) */
#define SQRT3_OVER_2     0.8660254038     /* sqrt(3)/2 */

#define SSI_RANGE        8191.0           /* 2^13 - 1  */
#define ENCODER_PPR      2048
#define ENCODER_CPR      ENCODER_PPR * 4  /* Encoder in X4 mode  */
#define INV_CPR          (1.0f / ENCODER_CPR)

#define MOTOR_POLE_PAIRS 10

extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim2;
extern PWM_data_pu pwm_data;

uint16_t ADC3_DMA_buff[ADC3_CH_NUM];

void FOC_motor_init(Motor_t *motor)
{
    motor->Ls_d = 0.0;
    motor->Ls_q = 0.0;
    motor->Rs = 1.0;
    motor->Phi_e = 0.0;
    motor->pole_pairs = 10;

    motor->I_scale = 350.0;
    motor->V_scale = 41.07;

    motor->dcBus_V = 1.0;
    motor->oneOverDcBus_invV = 1.0;

    motor->encoder_count = 0;
    motor->theta_e = 0.0;
    motor->omega_e = 0.0;

    motor->Sine = 0.0;
    motor->Cosine = 1.0;

    motor->Vff_dq_V[0] = 0.0;
    motor->Vff_dq_V[1] = 0.0;

    // Initialize the PI controllers
    motor->pi_id.Kp = 1.0;
    motor->pi_id.Ki = 0.0;
    motor->pi_id.Ui = 0.0;
    motor->pi_id.refValue = 0.0;
    motor->pi_id.fbackValue = 0.0;
    motor->pi_id.ffwdValue = 0.0;
    motor->pi_id.outMax = 1.0;
    motor->pi_id.outMin = -1.0;

    motor->pi_iq.Kp = 1.0;
    motor->pi_iq.Ki = 0.0;
    motor->pi_iq.Ui = 0.0;
    motor->pi_iq.refValue = 0.0;
    motor->pi_iq.fbackValue = 0.0;
    motor->pi_iq.ffwdValue = 0.0;
    motor->pi_iq.outMax = 1.0;
    motor->pi_iq.outMin = -1.0;
}

HAL_StatusTypeDef FOC_start_ADC_DMA(void)
{
	return HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3_DMA_buff, ADC3_CH_NUM);
}

FOC_Status_e FOC_encoder_init(Motor_t *motor)
{
	uint16_t SSI_angle;

	// Buffers MUST be uint16_t now
	uint16_t tx_buff[1] = { 0xFFFF };
	uint16_t rx_buff[1] = { 0 };

	// Note: 'Size' is now the number of 14-bit words.
	// We only need 1 word to get all 14 bits.
	SPI_TransmitReceive_Encoder((uint8_t*)tx_buff, (uint8_t*)rx_buff, 1);
	while(SPI_get_encoder_state() != HAL_SPI_STATE_READY)
	{
		;
	}

	// First reading is a garbage value
	// TODO: Find out why?
	SPI_TransmitReceive_Encoder((uint8_t*)tx_buff, (uint8_t*)rx_buff, 1);
	while(SPI_get_encoder_state() != HAL_SPI_STATE_READY)
	{
		;
	}
	// Remove START bit from 14b word to obtain 13b angle
	SSI_angle = rx_buff[0] & 0x1FFF;

	// 1. Convert to Mechanical Per-Unit (0.0 to 1.0)
	float mech_theta_pu = (float)SSI_angle / SSI_RANGE;

	// 2. Convert to Electrical Per-Unit (0.0 to 10.0)
	float elec_theta_pu_raw = mech_theta_pu * motor->pole_pairs;

	// 3. Wrap to [0.0, 1.0] range (Discard the integer revolutions)
	float elec_normalized = elec_theta_pu_raw - floorf(elec_theta_pu_raw);

	// 4. Map to CORDIC range (-1.0 to 1.0)
	// We want 0.0 to be 0, and 0.5 to be 1.0 (PI).
	// Anything > 0.5 (180 deg) must become negative to stay within the CORDIC's -PI to PI range.
	if (elec_normalized > 0.5f)
		motor->theta_e = (elec_normalized * 2.0f) - 2.0f;
	else
		motor->theta_e = (elec_normalized * 2.0f);

	motor->encoder_count = (uint32_t)((float)SSI_angle / SSI_RANGE * ENCODER_CPR + 0.5f);
	__HAL_TIM_SET_COUNTER(&htim2, motor->encoder_count);

	if(HAL_TIM_Base_Start(&htim2) != HAL_OK)
	{
		return FOC_INIT_FAULT;
	}
	return FOC_ALL_GOOD;
}

/**
 * @brief Map the value of variable "in" specified in the range between
 * in_min and in_max to a new range between out_min and out_max
 *
 * @param in      [in] - Input value
 * @param in_min  [in] - Low end of the input range
 * @param in_max  [in] - High end of the input range
 * @param out_min [in] - Low end of the output range
 * @param out_max [in] - High end of the output range
 *
 * @return float in mapped to a new range
 */
static inline float map(uint16_t in, uint16_t in_min, uint16_t in_max, float out_min, float out_max)
{
	return (float)(in - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

/**
 * @brief Run Clarke transform, to transform phase currents to
 * alpha-beta reference frame
 *
 * @param in [in/out] - Pointer to motor struct containing phase current data
 */
static inline void clarke_run(Motor_t *in)
{
    in->I_ab_A[0] = in->I_uvw_A[0];
    in->I_ab_A[1] = ((in->I_uvw_A[1] - in->I_uvw_A[2]) * ONE_OVER_SQRT3);
}

/**
 * @brief Run Park transform, to transform currents in the alpha-beta reference
 * frame to currents in the DQ reference frame
 *
 * @param in [in/out] - Pointer to motor struct containing the alpha-beta current data
 *
 * @note This function should be called after clarke_run()
 */
static inline void park_run(Motor_t *in)
{
    in->I_dq_A[0] = (in->I_ab_A[0] * in->Cosine) + (in->I_ab_A[1] * in->Sine);
    in->I_dq_A[1] = (in->I_ab_A[1] * in->Cosine) - (in->I_ab_A[0] * in->Sine);
}

/**
 * @brief Calculate the feed-forward corrective voltage from the measured
 * values Iq, Id and omega_e, as well as motor constants Ls_q, Ls_d and Phi_e
 *
 * @param in [in/out] - Pointer to motor struct for which feed-forward voltage
 * should be calculated
 */
static inline void decoupling_run(Motor_t *in)
{
    in->pi_id.ffwdValue = - in->I_dq_A[1] * in->Ls_q * in->omega_e;
    in->pi_iq.ffwdValue  = (in->I_dq_A[0] * in->Ls_d + in->Phi_e) * in->omega_e;
}

/**
 * @brief Limit the input value to a range between min and max
 *
 * @param in  [in] - Input value
 * @param max [in] - Maximum value
 * @param min [in] - Minimum value
 *
 * @return float input value limited to range between min and max
 */
static inline float MATH_sat(const float in, const float max, const float min)
{
	float out = in;

	out = fmaxf(out, min);
	out = fminf(out, max);

	return out;
}

/**
 * @brief Run the series version of PI controller
 *
 * @param pi         [in/out] - Pointer to PI regulator object
 * @param fbackValue [in]     - Feedback value (eg. measured Iq current)
 *
 * @return float - Corrective value to align feedback value to reference value
 *
 * @note This function should be called after updating the PI regulator's
 * reference value
 */
static inline float PI_run_series(PI_Obj * pi, const float fbackValue)
{
    float Error;
    float Up;
    float OutValue;

    Error = pi->refValue - fbackValue;

    // Compute the proportional output
    Up = pi->Kp * Error;

    // Compute the integral output with saturation
    pi->Ui = MATH_sat(pi->Ui + (pi->Ki * Up),pi->outMax,pi->outMin);

    // Saturate the output
    OutValue = MATH_sat(Up + pi->Ui + pi->ffwdValue,pi->outMax,pi->outMin);

    return OutValue;
}

/**
 * @brief Run the inverse Park transform to convert the corrective voltages in the DQ
 * reference frame to corrective voltages in the alpha-beta reference frame
 *
 * @param in [in/out] - Pointer to motor struct transform should be performed for
 *
 * @note This function should be called after the corrective phase voltages have been
 * calculated by running the feed-forward calculation followed by the PI regulator
 */
static inline void ipark_run(Motor_t *in)
{
    in->Vout_ab_V[0] = (in->Vout_dq_V[0] * in->Cosine) - (in->Vout_dq_V[1] * in->Sine);
    in->Vout_ab_V[1] = (in->Vout_dq_V[1] * in->Cosine) + (in->Vout_dq_V[0] * in->Sine);
}

/**
 * @brief Convert the corrective voltages in the alpha-beta reference frame to
 * duty-cycles for all 3 phases
 *
 * @param m [in/out] - Pointer to motor struct for which the duty-cycles should be calculated
 */
static inline void SVGEN_run(Motor_t *m)
{
    float Vmax_pu = 0,Vmin_pu = 0,Vcom_pu;

    float Va_pu    = m->Vout_ab_V[0] * m->oneOverDcBus_invV;
    float Vbeta_pu = m->Vout_ab_V[1] * m->oneOverDcBus_invV;

    float Va_tmp = (float)(0.5) * (-Va_pu);
    float Vb_tmp = SQRT3_OVER_2 * Vbeta_pu;

    // -0.5*Valpha + sqrt(3)/2 * Vbeta
    float Vb_pu = Va_tmp + Vb_tmp;

    // -0.5*Valpha - sqrt(3)/2 * Vbeta
    float Vc_pu = Va_tmp - Vb_tmp;

    Vmax_pu = fmaxf(fmaxf(Va_pu, Vb_pu), Vc_pu);
    Vmin_pu = fminf(fminf(Va_pu, Vb_pu), Vc_pu);

    // Compute Vcom = 0.5*(Vmax+Vmin)
    Vcom_pu = (float)0.5 * (Vmax_pu + Vmin_pu);

    // Subtract common-mode term to achieve SV modulation
    pwm_data.duty_cycle_uvw[0] = (Va_pu - Vcom_pu);
    pwm_data.duty_cycle_uvw[1] = (Vb_pu - Vcom_pu);
    pwm_data.duty_cycle_uvw[2] = (Vc_pu - Vcom_pu);
}

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
void FOC_run(Motor_t *mot, RampGen *rg)
{
	/* --- Calculate omega_e in rad/s (measured) ---------------------------------*/
	uint16_t new_encoder_count = __HAL_TIM_GET_COUNTER(&htim2);
	int16_t count_diff = (int16_t)(new_encoder_count - (uint16_t)mot->encoder_count);
	mot->encoder_count = (uint32_t)new_encoder_count;

	mot->omega_e = ((float)count_diff * INV_CPR) * mot->pole_pairs * TWO_PI * PWM_ISR_FREQUENCY;

	float mech_theta_pu = (float)mot->encoder_count * INV_CPR;
	float elec_norm = (mech_theta_pu * mot->pole_pairs);
	elec_norm -= floorf(elec_norm); // Wrap to [0, 1]

	// Map to [-1, 1] for CORDIC
	mot->theta_e = (elec_norm > 0.5f) ? (elec_norm * 2.0f - 2.0f) : (elec_norm * 2.0f);

	mot->Cosine = CORDIC_q31_cosf(mot->theta_e);
	mot->Sine   = CORDIC_q31_sinf(mot->theta_e);

//    mot->Vout_dq_V[0] = 0.5; // FOR TESTING -> open loop
//    mot->Vout_dq_V[1] = 0.5; // FOR TESTING -> open loop
//    mot->oneOverDcBus_invV = 1.0; // FOR TESTING -> open loop

	/* --- Read phase currents and convert them to Amperes -----------------------*/
    mot->I_uvw_A[0] = map(ADC3_DMA_buff[I_U_HI], 0, 4095, -350.0, 350.0);
	mot->I_uvw_A[1] = map(ADC3_DMA_buff[I_V_HI], 0, 4095, -350.0, 350.0);
	mot->I_uvw_A[2] = -mot->I_uvw_A[0] - mot->I_uvw_A[1];

	/* --- Read DC link voltage, convert to PU and calculate inverse -------------*/
	mot->dcBus_V = map(ADC3_DMA_buff[V_DC], 0, 4095, -350.0, 350.0); // TODO: Fix scaling
	mot->oneOverDcBus_invV = 1.0 / mot->dcBus_V;

	/* --- Generate Iq and Id request from analog input --------------------------*/
	mot->pi_id.refValue = 0.0; //For PMSM and no fiel-weakening Id should be 0A
	mot->pi_iq.refValue = AIN[0].percentage;

	/* --- Run Clarke-Park transforms to get Iq and Id (measured) ----------------*/
	clarke_run(mot);
	park_run(mot);

	/* --- Run decoupling and feed-forward to get correction voltage -------------*/
	decoupling_run(mot);

	/* --- Run PI control of Iq and Id to get correction voltage Vq and Vd -------*/
	mot->Vout_dq_V[0] = PI_run_series(&mot->pi_id, mot->I_dq_A[0]);
	mot->Vout_dq_V[1] = PI_run_series(&mot->pi_iq, mot->I_dq_A[1]);

    // Generate SVPWM
    ipark_run(mot);
    SVGEN_run(mot);

    PWM_timer_write_data_to_reg();
}
