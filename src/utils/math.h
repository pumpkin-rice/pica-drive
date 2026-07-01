/**
 * @file math.h
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_MOTOR_MATH_H_
#define _PICA_MOTOR_MATH_H_

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_TWOPI
#define M_TWOPI (M_PI * 2.0)
#endif

#define SQRT3               (1.732050808f) /*!< sqrt{3} */
#define FRAC_1_SQRT3        (0.577350269f) /*!< frac{1}{sqrt{3}} */
#define FRAC_2_SQRT3        (1.154700538f) /*!< frac{2}{sqrt{3}} */
#define FRAC_SQRT3_2        (SQRT3 * 0.5f) /*!< \frac{\sqrt{3}}{2}*/

#define constrain(_v, _l, _h)   ((_v) > (_h) ? (_h) : (_v) < (_l) ? (_l) : (_v))

#ifdef __cplusplus

#include <cmath>

namespace pica
{

template<typename T>
constexpr T square(T v)
{
    return v * v;
}

template<typename T>
constexpr T cubic(T v)
{
    return v*v*v;
}

template<typename T>
constexpr T sign_hard(T v)
{
    return std::signbit(v) ? (T)(-1) : (T)(1);
}

} // namespace pica

#else

#define sign_hard(_v) ((signbit(_v) ? (-1) : (1)))


#define deg2rad(_deg) (_deg * 0.017453293f)
#define rad2deg(_rad) (_rad * 57.295779513f)

#define rpm2rad_s(_rpm)     ((_rpm) / 60.f * 2 * M_PI)
#define rpm2deg_s(_rpm)     ((_rpm) * 6.f)

#define squre(_x)               ((_x) * (_x))
#define cubic(_v)               ((_v)*(_v)*(_v))

#endif

#ifdef __cplusplus
 extern "C"{
#endif

static inline float clampf(float v, float l, float h)
{
    return constrain(v, l, h);
}

// Wrap value to range.
// With default rounding mode (round to nearest),
// the result will be in range -y/2 to y/2
static inline float wrap_pm(float x, float y) {
#ifdef FPU_FPV4
    float intval = (float)round_int(x / y);
#else
    float intval = nearbyintf(x / y);
#endif
    return x - intval * y;
}

static inline float wrap_pm_pi(float rad)
{
    return wrap_pm(rad, 2*M_PI);
}

static inline float fmodf_pos(float x, float y)
{
    float res = wrap_pm(x, y);
    if (res < 0) {
        return res + y;
    }

    return res;
}

#ifdef __cplusplus
 }
#endif

#endif /* !_PICA_MOTOR_MATH_H_*/
