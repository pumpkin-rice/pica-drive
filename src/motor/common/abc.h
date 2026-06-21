/**
 * @file dq.h
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_MOTOR_ABC_H_
#define _PICA_MOTOR_ABC_H_

struct abc
{
    float a, b, c;
};

#define abc_reset(_abc) \
    ((_abc).a = (_abc).b = (_abc).c = 0)

#define abc_set(_abc, _a, _b, _c) \
    do { \
        (_abc).a = (_a); \
        (_abc).b = (_b); \
        (_abc).c = (_c); \
    } while(0)

#endif /*!_PICA_MOTOR_DQ_H_ */
