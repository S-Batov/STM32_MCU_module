/**
 * @file rampgen.h
 *
 * @brief Ramp generator used to step the motor rotor angle
 * signal on each TIM8_UP event
 */

#ifndef __RAMPGEN_H__
#define __RAMPGEN_H__

#include <stdint.h>

typedef struct {
	float Freq; 		  /* Input: Ramp frequency (pu)         */
	float StepAngleMax;   /* Parameter: Maximum step angle (pu) */
	float Angle;		  /* Variable: Step angle (pu)          */
	float Out;  	 	  /* Output: Ramp signal (pu)           */
} RampGen;

/**
 * @brief Compute the ramp generator angle
 *
 * @param rg[in/out] - Ramp generator pointer
 */
static inline void RampGen_step(RampGen *rg)
{
	// Compute the angle rate
	rg->Angle += rg->StepAngleMax * rg->Freq;

	// Saturate the angle rate within (-1,1)
	if(rg->Angle > 1.0)
	{
		rg->Angle -= 1.0;
	}
	else if(rg->Angle < -1.0)
	{
		rg->Angle += 1.0;
	}

	rg->Out = rg->Angle;
}

#endif /* _RAMPGEN_H_ */
