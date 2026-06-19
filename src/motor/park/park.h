/**
 * @file park.h
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-04-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _MOTOR_FOC_PARK_H_
#define _MOTOR_FOC_PARK_H_

#include "utils/math.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief 
 * 
 * @param[in] theta electrical angle, rad
 * @param[in] alpha 
 * @param[in] beta 
 * @param[in] d 
 * @param[in] q 
 */
static inline void park(float theta, float alpha, float beta, float *d, float *q)
{
	float c = cosf(theta);
	float s = sinf(theta);

	*d = ( c * alpha) + (s * beta);
	*q = (-s * alpha) + (c * beta);
}

static inline void ipark(float theta, float d, float q, float *alpha, float *beta)
{
	float c = cosf(theta);
	float s = sinf(theta);

	*alpha = (c * d) - (s * q);
	*beta  = s * d + c * q;
}

/**
 * @brief
 * 
 * @param[in] a 
 * @param[in] b 
 * @param[in] c -(a+b)
 * @param[in] alpha 
 * @param[in] beta 
 */
static inline void clarke(float a, float b, float c, float *alpha, float *beta)
{
	// *alpha = (2.f/3) * (a - 0.5f * b - 0.5f * c);
	// *beta  = (2.f/3) * (0.5f * SQRT3 * b - 0.5f * SQRT3 * c);

	*alpha = a;
	*beta = FRAC_1_SQRT3 * (b - c);
}

static inline void iclark(float alpha, float beta, float *a, float *b, float *c)
{
	*a = alpha;
	*b = -0.5*alpha + (0.5f * SQRT3) * beta;
	*c = -0.5*alpha - (0.5f * SQRT3) * beta;
}

#ifdef __cplusplus
 }
#endif

#endif /* !_MOTOR_FOC_PARK_H_ */
