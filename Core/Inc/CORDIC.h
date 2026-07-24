/**
 * @file CORDIC.h
 * @brief Function definitions for trigonometric functions using
 * CORDIC hardware accelerator
 */

#ifndef _CORDIC_H_
#define _CORDIC_H_

#include <math.h>

extern CORDIC_HandleTypeDef hcordic;

/**
 * @brief Convert a 32bits float to a Q1.31 notation integer
 * @param input [in] - floating point value to convert
 * @return a Q1.31 notation integer
 */
static inline int CORDIC_f32_to_q31(double input)
{
    const float Q31_MAX_F = 0x0.FFFFFFp0F;
    const float Q31_MIN_F = -1.0F;
    return (int)roundf(scalbnf(fmaxf(fminf(input, Q31_MAX_F), Q31_MIN_F), 31));
}

/**
 * @brief Convert a Q1.31 notation integer into a 32bits float
 * @param input [in] - a Q1.31 notation integer
 * @return floating point value to convert
 */
static inline float CORDIC_q31_to_f32(int input)
{
    return (float)input / 2147483648.0f;  // 2^31
}

/**
 * @brief Computes the trigonometric cosine function using the sordic accelerator using Q31 precision
 * @param x [in] - angle in radians divided by PI (-1 = -PI, 1 = PI)
 * @return the cosine of x
 */
static inline float CORDIC_q31_cosf(float x)
{
  CORDIC_ConfigTypeDef sConfig;
  int32_t input_q31 = CORDIC_f32_to_q31(x);
  int32_t output_q31;

  sConfig.Function = CORDIC_FUNCTION_COSINE;
  sConfig.Precision = CORDIC_PRECISION_6CYCLES;
  sConfig.Scale = CORDIC_SCALE_0;
  sConfig.NbWrite = CORDIC_NBWRITE_1;
  sConfig.NbRead = CORDIC_NBREAD_1;
  sConfig.InSize = CORDIC_INSIZE_32BITS;
  sConfig.OutSize = CORDIC_OUTSIZE_32BITS;
  HAL_CORDIC_Configure(&hcordic, &sConfig);

  HAL_CORDIC_CalculateZO(&hcordic, &input_q31, &output_q31, 1, 0);

  return CORDIC_q31_to_f32(output_q31);
}

/**
 * @brief Computes the trigonometric sine function using the cordic accelerator using Q31 precision
 * @param x [in] - angle in radians divided by PI (-1 = -PI, 1 = PI)
 * @return the sine of x
 */
static inline float CORDIC_q31_sinf(float x)
{
  CORDIC_ConfigTypeDef sConfig;
  int32_t input_q31 = CORDIC_f32_to_q31(x);
  int32_t output_q31;

  sConfig.Function = CORDIC_FUNCTION_SINE;
  sConfig.Precision = CORDIC_PRECISION_6CYCLES;
  sConfig.Scale = CORDIC_SCALE_0;
  sConfig.NbWrite = CORDIC_NBWRITE_1;
  sConfig.NbRead = CORDIC_NBREAD_1;
  sConfig.InSize = CORDIC_INSIZE_32BITS;
  sConfig.OutSize = CORDIC_OUTSIZE_32BITS;
  HAL_CORDIC_Configure(&hcordic, &sConfig);

  HAL_CORDIC_CalculateZO(&hcordic, &input_q31, &output_q31, 1, 0);

  return CORDIC_q31_to_f32(output_q31);
}

#endif /* _CORDIC_H_ */
