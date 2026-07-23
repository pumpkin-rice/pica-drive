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

#include "bldc/bldc.hpp"
#include "utils/math.h"
#include "current_controller_foc.hpp"

#include "drive_conf.h"

#include <algorithm>
#include <cmath>

namespace pica::motor::bldc {

bool FOC::init(void* cfg) {
    m_cfg = *reinterpret_cast<Config*>(cfg);

    if (BLDC::Type::kGimbal != m_motor.type()) {
        m_current_loop_enabled = true;
    }

    reset();

    updateGain();

    return true;
}

void FOC::updateGain() {
    float kp, ki;

    const auto& cfg = m_motor.m_cfg;

    // calculate current control gains
    kp = cfg.current_controller_bandwidth * cfg.phase_inductance;
    ki = cfg.current_controller_bandwidth * cfg.phase_resistance;

    // SPMSM，Kp、Ki 数值相同
    m_pi.kp.setAll(kp);
    m_pi.ki.setAll(ki);

    m_pi.integral.setAll(0.f);
}

void FOC::reset() {
    m_vdq_sp.zero();
    m_idq_sp.zero();
    m_idq_meas.zero();

    m_i_alpha_beta_meas.zero();

    m_idq_meas_filter_k = kFOCIDQMeasFilterKDefault;

    updateGain();
}

bool FOC::update(hrt_absnano now) {
    bool ret;

    const auto& cfg_motor = m_motor.m_cfg;

    float torque_sp = m_motor.m_torque_ref;

    float id = m_idq_sp(0);
    float iq = m_idq_sp(1);

    float current_limit = m_motor.m_effective_current_limit;

    if (!std::isfinite(torque_sp)) {
        return false;
    }

    if (BLDC::Type::kACIM == m_motor.type()) {
        // TODO: ACIM

    } else {
        id = std::clamp(id, -current_limit * 0.99f, current_limit * 0.99f);
    }

    // Convert requested torque to current
    if (BLDC::kACIM == m_motor.type()) {
    } else {
        iq = torque_sp / cfg_motor.torque_constant;
    }

    // 2-norm clamping where Id takes priority
    float iq_limit_sq = square(current_limit) - square(id);
    float iq_limit = (iq_limit_sq <= 0.f) ? 0.f : sqrtf(iq_limit_sq);
    iq = std::clamp(iq, -iq_limit, iq_limit);

    if (BLDC::kGimbal != m_motor.type()) {
        m_idq_sp = {id, iq};
    }

    // TODO: ACIM estimator update(timestamp)

    float vd = 0.f;
    float vq = 0.f;

    float omega = m_motor.m_velocity_est;

    if (cfg_motor.R_wL_FF_enabled) {
        float L = cfg_motor.phase_inductance;
        float Rs = cfg_motor.phase_resistance;

        if (!std::isfinite(omega)) {
        return -1;
        }

        vd -= omega * L * iq;
        vq += omega * L * id;

        vd += Rs * id;
        vq += Rs * iq;
    }

    if (cfg_motor.b_EMF_FF_enabled) {
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
        vq += omega * (2.f / 3.f) *
            (cfg_motor.torque_constant / cfg_motor.pole_pairs);
    }

    if (BLDC::kGimbal == cfg_motor.motor_type) {
        // reinterpret current as voltage
        m_vdq_sp = {vd + iq, vq + iq};

    } else {
        m_vdq_sp = {vd, vq};
    }

    m_ts_update = now;

    ret = true;

exit:
    return ret;
}

bool FOC::run(hrt_absnano ts_output, AlphaBeta* v_alpha_beta_final) {
    bool ret = false;

    float mod2vbus, vbus2mod; /*!< mod_dq 与 母线电压比例 */

    DQ idq;
    DQ mod_vdq;

    // 按照ns计算，uint32 最多在 4s
    uint32_t dt = hrt_diff_nano(&m_ts_update, &m_motor.timestampCurrentMeas());
    if (dt > PICA_CONTROLLER_LOOP_UPDATE_TO_CURRENT_MEAS_DELTA_MAX_NANO)
    {
        // 控制环更新延迟程序是否过大
        // TODO: error

        return false;
    }

    m_i_alpha_beta_meas = clarke(m_motor.m_current_meas);
    if (isfinite(m_i_alpha_beta_meas)) {
        float theta_now = m_motor.m_position_est +
                        m_motor.m_velocity_est * ((float)(dt) * 1e-9f);

        idq = park(m_i_alpha_beta_meas, theta_now);

        if (isfinite(idq)) {
        m_idq_meas += m_idq_meas_filter_k * (idq - m_idq_meas);
        }

    } else {
        m_idq_meas.zero();
    }

    mod2vbus = (2.f / 3.f) * m_motor.m_bus_voltage_meas;
    vbus2mod = 1.f / mod2vbus;

    if (m_current_loop_enabled) {
        float id_err, iq_err;
        float mod_scale_factor;

        if (!isfinite(idq)) {
        // TODO: error
        }

        DQ idq_err{m_idq_sp - idq};

        mod_vdq = {
            vbus2mod * (m_vdq_sp(0) + idq_err(0) * m_pi.kp(0) + m_pi.integral(0)),
            vbus2mod * (m_vdq_sp(1) + idq_err(1) * m_pi.kp(1) + m_pi.integral(1)),
        };

        // Vector modulation saturation, lock integrator if saturated
        // TODO make maximum modulation configurable
        mod_scale_factor = 0.8f * FRAC_SQRT3_2 * (1.f / mod_vdq.norm());

        if (mod_scale_factor < 1.f) {
            mod_vdq *= mod_scale_factor;

            m_pi.integral *= 0.99f;

            } else {
            m_pi.integral = {
                idq_err(0) * (m_pi.ki(0) * PICA_DRIVE_CURRENT_MEASURE_PERIOD),
                idq_err(1) * (m_pi.ki(1) * PICA_DRIVE_CURRENT_MEASURE_PERIOD),
            };
        }

        m_pi.err_prev = idq_err;

    } else {
        // 电压控制
        mod_vdq = vbus2mod * m_vdq_sp;
    }

    m_vdq_final = mod_vdq * mod2vbus;

    float theta_next = m_motor.m_position_est +
                        m_motor.m_velocity_est *
                            ((float)(ts_output - m_ts_update) * 1e-9f);
    AlphaBeta mod_valpha_beta = ipark(mod_vdq, theta_next);

    // genernate by svpwm
#if (PICA_DRIVE_PWM_GENERNATOR_SVPWM == 1)
    if (!m_motor.calcSVM(mod_valpha_beta))
    {
        // TODO: SVPWM generator error

        return false;
    }
#endif

    // genernate by iclarke
#if (PICA_DRIVE_PWM_GENERNATOR_ICLARKE == 1)
    iclarke(mod_valpha_beta).copyTo(m_motor.m_duty_cycle);
#endif

    *v_alpha_beta_final = mod2vbus * mod_valpha_beta;

    if (isfinite(idq)) {
        m_motor.m_bus_current_meas = mod_vdq * idq;
        m_motor.m_power_watt =
            m_motor.m_bus_voltage_meas * m_motor.m_bus_current_meas;
    }

    ret = true;

exit:
    return ret;
}

}  // namespace pica::motor::bldc
