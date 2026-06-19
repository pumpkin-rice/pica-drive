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

#ifndef _PICA_MOTOR_DQ_H_
#define _PICA_MOTOR_DQ_H_

struct dq
{
    float d;
    float q;
};

#define dq_reset(_dq) \
    ((_dq).d = (_dq).q = 0)

#define dq_set(_dq, _value_d, _value_q) \
    do { \
        (_dq).d = (_value_d), \
        (_dq).q = (_value_q), \
    } while(0)

#endif /*!_PICA_MOTOR_DQ_H_ */
