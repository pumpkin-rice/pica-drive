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

#ifndef _PICA_MOTOR_ALPHA_BETA_H_
#define _PICA_MOTOR_ALPHA_BETA_H_

struct alpha_beta
{
    float alpha;
    float beta;
};

#define alpha_beta_reset(_ab) \
        ((_ab).alpha = (_ab).beta = 0)

#define alpha_beta_set(_ab, _value_alpha, _value_beta) \
    do { \
        (_ab).alpha = (_value_alpha), \
        (_ab).beta = (_value_beta), \
    } while(0)

#endif /*!_PICA_MOTOR_DQ_H_ */
