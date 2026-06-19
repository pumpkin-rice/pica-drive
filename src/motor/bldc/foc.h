/**
 * @file controller.h
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_MOTOR_CURRENT_CONTROLLER_H_
#define _PICA_MOTOR_CURRENT_CONTROLLER_H_

#include "motor/current_controller.h"
#include "motor/common/alpha_beta.h"
#include "motor/common/dq.h"

#include <stdint.h>
#include <stdbool.h>

#define FOC_IDQ_MEAS_FILTER_K_DEFAULT_VAL       (1.f)

struct foc
{
    CURRENT_CONTROLLER_OBJECT;

    float vd_sp, vq_sp; /*!< V */
    float id_sp, iq_sp; /*!< A */
    float id_meas, iq_meas; /*!< A */

    float i_alpha_meas, i_beta_meas; /*!< A */
    float v_alpha_final, v_beta_final; /*!< V */

    float idq_meas_filter_k;

    struct {
        float kp;
        float ki, integral, integral_limit;
        float output_limit;

        float err_prev, output_prev;
    } id_pi, iq_pi;
};

#define curr_obj2foc(_curr)  ((struct foc *)(_curr))

#endif /* !_PICA_MOTOR_CURRENT_CONTROLLER_H_ */
