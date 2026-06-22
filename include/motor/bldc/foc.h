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

#include <stdint.h>
#include <stdbool.h>

#define FOC_IDQ_MEAS_FILTER_K_DEFAULT_VAL       (1.f)

struct foc
{
    CURRENT_CONTROLLER_OBJECT;

    float vdq_sp[2]; /*!< V */
    float idq_sp[2]; /*!< A */
    
    float idq_meas[2]; /*!< A */
    float i_alpha_beta_meas[2]; /*!< A */

    float vdq_final[2];
    float v_alpha_beta_final[2]; /*!< V */

    float idq_meas_filter_k;

    struct {
        float kp;
        float ki, integral, integral_limit;
        float output_limit;

        float err_prev, output_prev;
    } id_pi, iq_pi;
};

#define curr_obj2foc(_curr)  ((struct foc *)(_curr))

/*********************** 调试接口 ********************************/
#if ENABLE_PICA_DRIVE_DEBUG == 1

inline float *foc_dbg_vdq(struct foc *foc)
{
    return foc->vdq_sp;
}

inline float *foc_dbg_idq_meas(struct foc *foc)
{
    return foc->idq_meas;
}

inline float *foc_dbg_i_alpha_beta_measured(struct foc *foc)
{
    return foc->i_alpha_beta_meas;
}

inline float *foc_dbg_vdq_final(struct foc *foc)
{
    return foc->vdq_final;
}

inline float *foc_dbg_v_alpha_beta_final(struct foc *foc)
{
    return foc->v_alpha_beta_final;
}

#endif /*!ENABLE_PICA_DRIVE_EDBUG */

#endif /* !_PICA_MOTOR_CURRENT_CONTROLLER_H_ */
