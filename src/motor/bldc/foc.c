/**
 * @file foc.c
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "motor/park/park.h"
#include "motor/pwm/svm.h"

#include "bldc.h"
#include "motor/current_controller.h"

#include "private/math.h"

#include <string.h>
#include <assert.h>

static int foc_update(current_controller_obj *obj);

/**
 * @brief run current control loop
 * 
 * @param[in] obj
 * @param[in] time2last_sampling time to previous current sampling, second
 * @param[in] time2next_output time to next pwm output, second
 * @return int 
 */
static int foc_run(current_controller_obj *obj, float time2last_sampling, float time2next_output);

static int foc_update_controller_gain(current_controller_obj *ctrl)
{
    struct foc *foc = curr_obj2foc(ctrl);
    struct motor_parameters *params = foc->motor->param;

    float kp, ki;

    // calculate current control gains
    kp = params->current_controller_bandwidth * params->phase_inductance;
    ki = params->current_controller_bandwidth * params->phase_resistance;

    foc->id_pi.ki = ki;
    foc->id_pi.kp = kp;
    foc->id_pi.integral = 0;

    foc->iq_pi.ki = ki;
    foc->iq_pi.kp = kp;
    foc->iq_pi.integral = 0;
}

static void reset_parameter(struct foc *foc)
{
    foc->idq_meas[0] = foc->idq_meas[1] = 0.f;
    foc->idq_sp[0] = foc->idq_sp[1] = 0.f;
    foc->vdq_sp[0] = foc->vdq_sp[1] = 0.f;

    foc->i_alpha_beta_meas[0] = foc->i_alpha_beta_meas[1] = 0.f;
    foc->v_alpha_beta_final[0] = foc->v_alpha_beta_final[1] = 0.f;

    foc->idq_meas_filter_k = FOC_IDQ_MEAS_FILTER_K_DEFAULT_VAL;
}

int foc_init(current_controller_obj *obj, motor_obj *motor)
{
    struct foc *foc = curr_obj2foc(obj);

    const struct bldc *bldc = motor2bldc(motor);
    const struct motor_parameters *params = bldc->param;

    foc->motor = motor;

    foc->run = foc_run;
    foc->update = foc_update;
    foc->update_config = foc_update_controller_gain;

    foc->current_loop_enabled = (params->type != MOTOR_TYPE_GIMBAL);

    reset_parameter(foc);
    foc_update_controller_gain(obj);

    return 0;
}

int foc_reset(current_controller_obj *obj, motor_obj *motor)
{
    struct foc *foc = curr_obj2foc(obj);

    const struct bldc *bldc = motor2bldc(obj->motor);
    struct motor_parameters *params = motor->param;

    foc->motor = motor;

    foc->run = foc_run;
    foc->update = foc_update;
    foc->update_config = foc_update_controller_gain;

    foc->current_loop_enabled = (params->type != MOTOR_TYPE_GIMBAL);

    reset_parameter(foc);
    foc_update_controller_gain(obj);
}

/**
 * @brief 由力矩目标值生成 dq 电流
 * 
 * @param[in] ctrl 
 * @return int 
 */
int foc_update(current_controller_obj *obj)
{
    struct foc *foc = curr_obj2foc(obj);
    struct bldc *bldc = motor2bldc(obj->motor);
    struct motor_parameters *param = bldc->param;

    // Load setpoints from previous iteration.
    float id = foc->idq_sp[0];
    float iq = foc->idq_sp[1];
    
    float current_limit = bldc->effective_current_limit;

    float torque = bldc->torque_sp;

    int ret;

    // convert torque setpoint to motor directon
    if (!isfinite(torque)) {
        return -1;
    }

    // Autoflux tracks old Iq (that may be 2-norm clamped last cycle) to make sure we are chasing a feasable current.
    if (MOTOR_TYPE_ACIM == param->type
            && param->acim_autoflux_enabled) {
        // TODO: ACIM

    } else {
        // 1% space reserved for Iq to avoid numerical issues
        id = clamp(id, -current_limit * 0.99f, current_limit * 0.99f);
    }

    // Convert requested torque to current
    if (MOTOR_TYPE_ACIM == param->type) {

    } else {
        iq = torque / param->torque_constant;
    }

    // 2-norm clamping where Id takes priority
    float iq_limit_sq = squre(current_limit) - squre(id);
    float iq_limit = (iq_limit_sq <= 0.f) ? 0.f : sqrtf(iq_limit_sq);
    iq = clamp(iq, -iq_limit, iq_limit);

    if (MOTOR_TYPE_GIMBAL != param->type) {
        foc->idq_sp[0] = id;
        foc->idq_sp[1] = iq;
    }

    // TODO: ACIM estimator update(timestamp)

    float vd = 0.f;
    float vq = 0.f;

    float omega = bldc->omega_elec;

    if (param->R_wL_FF_enabled) {
        float L  = param->phase_inductance;
        float Rs = param->phase_resistance;

        if (!isfinite(omega)) {
            return -1;
        }

        vd -= omega * L * iq;
        vq += omega * L * id;

        vd += Rs * id;
        vq += Rs * iq;
    }

    if (param->b_EMF_FF_enabled) {
        if (!isfinite(omega)) {
            // TODO: error unknow omega
            return -1;
        }

        /**
         * @brief 反电动势补偿
         * 
         * $EMF = \omega * K_e = \omega * \psi_f = \omega * K_t / p_n$
         *
         */
        vq += omega * (2.f/3.f) * (param->torque_constant / param->pole_pairs);
    }

    if (MOTOR_TYPE_GIMBAL == param->type) {
        // reinterpret current as voltage
        foc->vdq_sp[0] = vd + id;
        foc->vdq_sp[1] = vq +iq;

    } else {
        foc->vdq_sp[0] = vd;
        foc->vdq_sp[1] = vq;
    }
}

static inline float update_dq_filter(float meas, float filted, float gain)
{
    return gain * (meas - filted) + filted;
}

int foc_run(current_controller_obj *obj, float time2last_sampling, float time2next_output)
{
    struct foc *foc = curr_obj2foc(obj);
    struct bldc *bldc = motor2bldc(foc->motor);

    float i_alpha_meas, i_beta_meas; /*!< 由 abc 电流计算得到的 alpha beta 电流 */
    float id, iq; /*!< 由 abc 电流计算得到的 dq 电流 */
    bool idq_usable = false;

    float mod2vbus, vbus2mod; /*!< mod_dq 与 母线电压比例 */
    float mod_vd, mod_vq; /*!< dq output voltage */
    float mod_vbeta, mod_valpha; /*!< get from ipark(mod_vd, mod_vq) */

    int ret;

    clarke(bldc->current_phases_meas[0],
        bldc->current_phases_meas[1],
        bldc->current_phases_meas[2],
        &i_alpha_meas,
        &i_beta_meas
    );

    foc->i_alpha_beta_meas[0] = i_alpha_meas;
    foc->i_alpha_beta_meas[1] = i_beta_meas;

    if (isfinite(i_alpha_meas) && isfinite(i_beta_meas)) {
        float theta_now = bldc->theta_elec
                            + bldc->omega_elec * time2last_sampling;

        park(theta_now, i_alpha_meas, i_beta_meas, &id, &iq);

        if (isfinite(i_alpha_meas) && isfinite(i_beta_meas)) {
            idq_usable = true;

            foc->idq_meas[0] +=
                    foc->idq_meas_filter_k * (id - foc->idq_meas[0]);
            foc->idq_meas[1] +=
                    foc->idq_meas_filter_k * (iq - foc->idq_meas[1]);
        }

    } else {
        foc->idq_meas[0] = foc->idq_meas[1] = 0.f;
    }

    mod2vbus = (2.f/3.f) * bldc->bus_voltage_meas; /*!< 等幅变换 */
    vbus2mod = 1/mod2vbus;

    if (foc->current_loop_enabled) {
        typeof(foc->id_pi) *pi_d = &foc->id_pi;
        typeof(foc->iq_pi) *pi_q = &foc->iq_pi;

        float id_err, iq_err;

        float mod_scale_factor;

        if (!idq_usable) {
            // TODO: error
        }

        id_err = foc->idq_sp[0] - id;
        iq_err = foc->idq_sp[1] - iq;

        mod_vd = vbus2mod * (foc->vdq_sp[0]
            + id_err * pi_d->kp + pi_d->integral);
        mod_vq = vbus2mod * (foc->vdq_sp[1]
            + iq_err * pi_q->kp + pi_q->integral);

        // 计算理论最大值
        // mod_scale_factor = FRAC_SQRT3_2 * (1.f / sqrtf(mod_vd * mod_vd + mod_vq * mod_vq));
        // // 对最大值进行限幅
        // mod_scale_factor *= 0.8f;

        // Vector modulation saturation, lock integrator if saturated
        // TODO make maximum modulation configurable
        mod_scale_factor = 0.8f * FRAC_SQRT3_2 * (1.f / sqrtf(mod_vd * mod_vd + mod_vq * mod_vq));
        if (mod_scale_factor < 1.f) {
            mod_vd *= mod_scale_factor;
            mod_vq *= mod_scale_factor;

            pi_d->integral *= 0.99f;
            pi_q->integral *= 0.99f;

        } else {
            pi_d->integral += id_err * (pi_d->ki * time2last_sampling);
            pi_q->integral += iq_err * (pi_q->ki * time2last_sampling);
        }

    } else {
        // voltage control mode

        // V/F 控制，电流开环，通过速度环完成闭环
        mod_vd = vbus2mod * foc->vdq_sp[0];
        mod_vq = vbus2mod * foc->vdq_sp[1];
    }

    {
        float theta_next = bldc->theta_elec + bldc->omega_elec * time2next_output;
        ipark(theta_next, mod_vd, mod_vq, &mod_valpha, &mod_vbeta);
    }

    // genernate svpwm
    // if (!svm(mod_valpha, mod_vbeta,
    //         &bldc->duty_cycle[0],
    //         &bldc->duty_cycle[1],
    //         &bldc->duty_cycle[2]
    //     )
    // ) {
    // 	// TODO: SVPWM generator error

    // }
    iclark(mod_valpha, mod_vbeta,
            &bldc->duty_cycle[0],
            &bldc->duty_cycle[1],
            &bldc->duty_cycle[2]
        );

    foc->vdq_final[0] = mod2vbus * mod_vd;
    foc->vdq_final[1] = mod2vbus * mod_vq;

    // Report final applied voltage in stationary frame (for sensorless estimator)
    foc->v_alpha_beta_final[0] = mod2vbus * mod_valpha;
    foc->v_alpha_beta_final[1]  = mod2vbus * mod_vbeta;

    if (idq_usable) {
        bldc->bus_current_meas = mod_vd * id + mod_vq * iq;
        bldc->power_watt = bldc->bus_voltage_meas * bldc->bus_current_meas;
    }

    return 0;
}
