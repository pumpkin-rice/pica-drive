/**
 * @file foc.cc
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "foc.hpp"
#include "motor/bldc.hpp"
#include "motor/pwm/svm.h"
#include "utils/math.h"

#include <cmath>
#include <algorithm>

using namespace pica::motor;

bool FOC::init(const Config *cfg)
{
    CurrentController::init(cfg);

    if (BLDC::GIMBAL != cfg->motor_type) {
        m_current_loop_enabled = true;
    }

    reset();

    updateGain();

    return true;
}

void FOC::updateGain()
{
    float kp, ki;

    // calculate current control gains
    kp = m_cfg->current_controller_bandwidth * m_cfg->phase_inductance;
    ki = m_cfg->current_controller_bandwidth * m_cfg->phase_resistance;

    m_pi_id.ki = ki;
    m_pi_id.kp = kp;
    m_pi_id.integral = 0;

    m_pi_iq.ki = ki;
    m_pi_iq.kp = kp;
    m_pi_iq.integral = 0;
}

void FOC::reset()
{
    m_vdq_sp.reset();
    m_idq_sp.reset();
    m_idq_meas.reset();

    m_i_alpha_beta_meas.reset();
    m_v_alpha_beta_final.reset();

    m_idq_meas_filter_k = kFOCIDQMeasFilterKDefault;

    updateGain();
}

bool FOC::generateCurrentSetpoint()
{
    bool ret;

    BLDC& bldc = *dynamic_cast<BLDC *>(&m_motor);

    float torque_sp = bldc.m_torque_cmd;

    float id = m_idq_sp.d;
    float iq = m_idq_sp.q;

    float current_limit = bldc.m_effective_current_limit;

    if (!std::isfinite(bldc.m_torque_sp)) {
        return false;
    }

    if (BLDC::ACIM == bldc.getMotorType()) {
        // TODO: ACIM

    } else {
        id = std::clamp(id, -current_limit * 0.99f, current_limit * 0.99f);
    }

    // Convert requested torque to current
    if (BLDC::ACIM == bldc.getMotorType()) {

    } else {
        iq = bldc.m_torque_sp / m_cfg->torque_constant;
    }

    // 2-norm clamping where Id takes priority
    float iq_limit_sq = square(current_limit) - square(id);
    float iq_limit = (iq_limit_sq <= 0.f) ? 0.f : sqrtf(iq_limit_sq);
    iq = std::clamp(iq, -iq_limit, iq_limit);

    if (BLDC::GIMBAL != bldc.getMotorType()) {
        m_idq_sp.set(id, iq);
    }

    // TODO: ACIM estimator update(timestamp)

    float vd = 0.f;
    float vq = 0.f;

    float omega = bldc.getElectricalVelocityEst();

    if (m_cfg->R_wL_FF_enabled) {
        float L  = m_cfg->phase_inductance;
        float Rs = m_cfg->phase_resistance;

        if (!std::isfinite(omega)) {
            return -1;
        }

        vd -= omega * L * iq;
        vq += omega * L * id;

        vd += Rs * id;
        vq += Rs * iq;
    }

    if (m_cfg->b_EMF_FF_enabled) {
        if (!std::isfinite(omega)) {
            // TODO: error unknow omega
            goto exit;
        }

        /**
         * @brief 反电动势补偿
         * 
         * $EMF = \omega * K_e = \omega * \psi_f = \omega * K_t / p_n$
         *
         */
        vq += omega * (2.f/3.f) * (m_cfg->torque_constant / m_cfg->pole_pairs);
    }

    if (BLDC::GIMBAL == m_cfg->motor_type) {
        // reinterpret current as voltage
        m_vdq_sp.set(vd+iq, vq+iq);

    } else {
        m_vdq_sp.set(vd, vq);
    }

    ret = true;

exit:
    return ret;
}

bool FOC::update()
{
    generateCurrentSetpoint();

    return true;
}

bool FOC::Run(CurrentController *ctrl,
                    float time2last_meas, float time2next_pwm_output,
                    float period)
{
    bool ret = false;

    FOC& foc = *dynamic_cast<FOC *>(ctrl);
    BLDC& bldc = *dynamic_cast<BLDC *>(&foc.m_motor);

    float mod2vbus, vbus2mod; /*!< mod_dq 与 母线电压比例 */

    DQ idq;
    float mod_vd, mod_vq;
    
    foc.m_i_alpha_beta_meas.clarke(bldc.m_current_meas);
    if (foc.m_i_alpha_beta_meas.isValid()) {
        float theta_now = bldc.getElectricalPositionEst()
                        + bldc.getElectricalVelocityEst() * time2last_meas;
        
        idq.park(foc.m_i_alpha_beta_meas, theta_now);

        if (idq.isValid()) {
            foc.m_idq_meas.d +=
                foc.m_idq_meas_filter_k * (idq.d - foc.m_idq_meas.d);
            foc.m_idq_meas.q +=
                foc.m_idq_meas_filter_k * (idq.q - foc.m_idq_meas.q);
        }

    } else {
        foc.m_idq_meas.reset();
    }

    mod2vbus = (2.f/3.f) * bldc.m_bus_voltage_meas;
    vbus2mod = 1.f/mod2vbus;

    if (foc.m_current_loop_enabled) {
        ControllerPI& controller_d = foc.m_pi_id;
        ControllerPI& controller_q = foc.m_pi_iq;

        float id_err, iq_err;
        float mod_scale_factor;

        if (!idq.isValid()) {
            // TODO: error
        }

        id_err = foc.m_idq_sp.d - idq.d;
        iq_err = foc.m_idq_sp.q - idq.q;

        mod_vd = vbus2mod * (foc.m_vdq_sp.d
                    + id_err * controller_d.kp + controller_d.integral);
        mod_vq = vbus2mod * (foc.m_vdq_sp.q
                    + iq_err * controller_q.kp + controller_q.integral);
        
        // Vector modulation saturation, lock integrator if saturated
        // TODO make maximum modulation configurable
        mod_scale_factor = 0.8f * FRAC_SQRT3_2 * (1.f / sqrtf(mod_vd * mod_vd + mod_vq * mod_vq));

        if (mod_scale_factor < 1.f) {
            mod_vd *= mod_scale_factor;
            mod_vq *= mod_scale_factor;

            controller_d.integral *= 0.99f;
            controller_q.integral *= 0.99f;

        } else {
            controller_d.integral += id_err * (controller_d.ki * period);
            controller_q.integral += iq_err * (controller_q.ki * period);
        }

        controller_d.err_prev = id_err;
        controller_q.err_prev = iq_err;

    } else {
        // 电压控制
        mod_vd = vbus2mod * foc.m_vdq_sp.d;
        mod_vq = vbus2mod * foc.m_vdq_sp.q;
    }

    float theta_next = bldc.getElectricalPositionEst()
                + bldc.getElectricalVelocityEst() * time2next_pwm_output;
    AlphaBeta mod_valpha_beta{mod_vd, mod_vq, theta_next};

    // genernate svpwm
    // if (!svm(mod_valpha_beta.alpha, mod_valpha_beta.beta,
    //             &bldc.m_duty_cycle.a,
    //             &bldc.m_duty_cycle.b,
    //             &bldc.m_duty_cycle.c))
    // {
    //     // TODO: SVPWM generator error
    // }

    bldc.m_duty_cycle.iclarke(mod_valpha_beta);

    foc.m_v_alpha_beta_final.alpha = mod2vbus * mod_valpha_beta.alpha;
    foc.m_v_alpha_beta_final.beta  = mod2vbus * mod_valpha_beta.beta;

    if (idq.isValid()) {
        bldc.m_bus_current_meas = mod_vd * idq.d + mod_vq * idq.q;
        bldc.m_power_watt = bldc.m_bus_voltage_meas * bldc.m_bus_current_meas;
    }

    ret = true;

exit:
    return ret;
}
