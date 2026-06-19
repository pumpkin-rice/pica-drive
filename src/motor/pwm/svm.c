/**
 * @file svm.c
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-04-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "svm.h"

#define FRAC_1_SQRT3    (0.577350269f) /*!< frac{1}{sqrt{3}} */
#define FRAC_2_SQRT3    (1.154700538f) /*!< frac{2}{sqrt{3}} */

bool svm(float alpha, float beta, float *a, float *b, float *c)
{
	float tA, tB, tC;
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (FRAC_1_SQRT3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2
        } else {
            //quadrant II
            if (-FRAC_1_SQRT3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-FRAC_1_SQRT3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (FRAC_1_SQRT3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - FRAC_1_SQRT3 * beta;
            float t2 = FRAC_2_SQRT3 * beta;

            // PWM timings
            tA = (1.0f - t1 - t2) * 0.5f;
            tB = tA + t1;
            tC = tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + FRAC_1_SQRT3 * beta;
            float t3 = -alpha + FRAC_1_SQRT3 * beta;

            // PWM timings
            tB = (1.0f - t2 - t3) * 0.5f;
            tA = tB + t3;
            tC = tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = FRAC_2_SQRT3 * beta;
            float t4 = -alpha - FRAC_1_SQRT3 * beta;

            // PWM timings
            tB = (1.0f - t3 - t4) * 0.5f;
            tC = tB + t3;
            tA = tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + FRAC_1_SQRT3 * beta;
            float t5 = -FRAC_2_SQRT3 * beta;

            // PWM timings
            tC = (1.0f - t4 - t5) * 0.5f;
            tB = tC + t5;
            tA = tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - FRAC_1_SQRT3 * beta;
            float t6 = alpha - FRAC_1_SQRT3 * beta;

            // PWM timings
            tC = (1.0f - t5 - t6) * 0.5f;
            tA = tC + t5;
            tB = tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -FRAC_2_SQRT3* beta;
            float t1 = alpha + FRAC_1_SQRT3 * beta;

            // PWM timings
            tA = (1.0f - t6 - t1) * 0.5f;
            tC = tA + t1;
            tB = tC + t6;
        } break;
    }

    bool result_valid =
            tA >= 0.0f && tA <= 1.0f
         && tB >= 0.0f && tB <= 1.0f
         && tC >= 0.0f && tC <= 1.0f;

    *a = tA;
    *b = tB;
    *c = tC;
    
    return result_valid;
}
