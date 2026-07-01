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

namespace pica::motor
{

struct AlphaBeta;
struct DQ;

struct ThreePhase
{
    union {
        struct {
            float a;
            float b;
            float c;
        };
        float raw[3] = {0.f,0.f,0.f};
    };

    ThreePhase() = default;
    ThreePhase(float va, float vb, float vc) : a(va), b(vb), c(vc) {}

    void reset() { a = b = c = 0.f; }

    const ThreePhase& set(float va, float vb, float vc);

    constexpr float *get() { return raw; }

    const ThreePhase& iclarke(const AlphaBeta& ab);

    bool isValid()
    {
        return std::isfinite(a)
                && std::isfinite(b)
                && std::isfinite(c);
    }
};

struct AlphaBeta
{
    float alpha{0}, beta{0};

    AlphaBeta() = default;

    AlphaBeta(const ThreePhase& abc)
    {
        clarke(abc);
    }

    AlphaBeta(const DQ& dq, float theta);

    AlphaBeta(float d, float q, float theta)
    {
        ipark(d, q, theta);
    }

    const AlphaBeta& set(float a, float b)
    {
        alpha = a;
        beta = b;

        return *this;
    }

    void reset()
    {
        set(0.f, 0.f);
    }

    const AlphaBeta& clarke(const ThreePhase& abc)
    {
        alpha = abc.a;
        beta = FRAC_1_SQRT3 * (abc.b - abc.c);

        return *this;
    }

    const AlphaBeta& ipark(const DQ& dq, float theta);

    const AlphaBeta& ipark(float d, float q, float theta)
    {
        float c = cosf(theta);
        float s = sinf(theta);

        alpha = (c * d) - (s * q);
        beta  = (s * d) + (c * q);

        return *this;
    }

    bool isValid()
    {
        return std::isfinite(alpha) && std::isfinite(beta);
    }
};

struct DQ
{
    float d{0}, q{0};

    DQ() = default;

    DQ(const AlphaBeta&ab, float theta)
    {
        park(ab, theta);
    }

    const DQ& set(float vd, float vq)
    {
        d = vd;
        q = vq;

        return *this;
    }

    constexpr void reset()
    {
        d = q = 0.f;
    }

    const DQ& park(const AlphaBeta&ab, float theta)
    {
        float c = cosf(theta);
        float s = sinf(theta);

        d = ( c * ab.alpha) + (s * ab.beta);
        q = (-s * ab.alpha) + (c * ab.beta);

        return *this;
    }

    bool isValid()
    {
        return std::isfinite(d) && std::isfinite(q);
    }
};

inline const ThreePhase& ThreePhase::set(float va, float vb, float vc)
{
    a = va;
    b = vb;
    c = vc;

    return *this;
}

inline const ThreePhase& ThreePhase::iclarke(const AlphaBeta& ab)
{
    a = ab.alpha;
    b = -0.5*ab.alpha + (0.5f * SQRT3) * ab.beta;
    c = -0.5*ab.alpha - (0.5f * SQRT3) * ab.beta;

    return *this;
}

inline AlphaBeta::AlphaBeta(const DQ& dq, float theta)
{
    ipark(dq, theta);
}

inline const AlphaBeta& AlphaBeta::ipark(const DQ& dq, float theta)
{
    float c = cosf(theta);
    float s = sinf(theta);

    alpha = (c * dq.d) - (s * dq.q);
    beta  = (s * dq.d) + (c * dq.q);

    return *this;
}

}

#endif

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
