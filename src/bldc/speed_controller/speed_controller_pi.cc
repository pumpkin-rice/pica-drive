/**
 * @file PI.cc
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-02
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bldc/bldc.hpp"

#include <cmath>
#include <algorithm>

using namespace pica::motor::bldc;

static inline float limitVelocity(const float vel_est, const float vel_gain, const float vel_limit, const float torque)
{
    float Tmax = vel_gain * (vel_limit - vel_est);
    float Tmin = vel_gain * (-vel_limit - vel_est);
    
    return std::clamp(torque, Tmin, Tmax);
}

bool SpeedControllerPI::init(void *cfg)
{
    m_cfg = *reinterpret_cast<Config*>(cfg);

    return true;
}

bool SpeedControllerPI::update(float *torque_ref, hrt_absnano now)
{
    const auto& cfg_motor = m_motor.m_cfg;

    float vel_est = m_motor.m_velocity_est;
    float vel_des = m_motor.m_velocity_sp;
    float torque = m_motor.m_torque_sp;
    const float Tlim = m_motor.getMaxAvailableTorque();

    // 位置控制
    float gain_scheduling_multiplier = 1.0f;

    float vel_lim = cfg_motor.vel_limit;
    float vel_gain = m_cfg.vel_gain;
    float vel_integrator_gain = m_cfg.vel_integrator_gain;

    if (Motor::ControlMode::kPosition <= m_motor.getControlMode()) {
        float pos_err = m_motor.m_position_sp - m_motor.m_position_est;
        if (!std::isfinite(pos_err)) {
            // TODO: error

            return false;
        }

        vel_des += m_cfg.pos_gain * pos_err;

        // V 形增益调节器
        float pos_err_abs = std::abs(pos_err);
        if (m_cfg.gain_scheduling_enabled
                && pos_err_abs <= m_cfg.gain_scheduling_width)
        {
            gain_scheduling_multiplier
                    = pos_err_abs / m_cfg.gain_scheduling_width;
        }
    }

    // Velocity limiting
    if (cfg_motor.vel_limit_enabled) {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    if (cfg_motor.overspeed_error_enabled) {
        if (!std::isfinite(vel_est)) {
            // TODO: error
            return false;
        }

        if (std::abs(vel_est) > cfg_motor.vel_limit_tolerance * vel_lim) {
            // TODO: error overrspeed
            return false;
        }
    }

    // TODO: ACIM 弱磁控制
    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM)
    // float vel_gain = config_.vel_gain;
    // float vel_integrator_gain = config_.vel_integrator_gain;
    // if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
    //     float effective_flux = axis_->acim_estimator_.rotor_flux_;
    //     float minflux = axis_->motor_.config_.acim_gain_min_flux;
    //     if (std::abs(effective_flux) < minflux)
    //         effective_flux = std::copysignf(minflux, effective_flux);
    //     vel_gain /= effective_flux;
    //     vel_integrator_gain /= effective_flux;
    //     // TODO: also scale the integral value which is also changing units.
    //     // (or again just do control in torque units)
    // }

    // TODO: 抗齿槽转矩
    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    // if (anticogging_valid_ && config_.anticogging.anticogging_enabled) {
    //     if (!anticogging_pos_estimate.has_value()) {
    //         set_error(ERROR_INVALID_ESTIMATE);
    //         return false;
    //     }
    //     float anticogging_pos = *anticogging_pos_estimate / axis_->encoder_.getCoggingRatio();
    //     torque += config_.anticogging.cogging_map[std::clamp(mod((int)anticogging_pos, 3600), 0, 3600)];
    // }

    float vel_err = 0.f;
    if (Motor::ControlMode::kVelocity <= m_motor.getControlMode()) {
        if (!std::isfinite(vel_est)) {
            // TODO: error
            return false;
        }

        vel_err = vel_des - vel_est;
        torque += (vel_gain * gain_scheduling_multiplier) * vel_err;

        // Velocity integral action before limiting
        torque += m_vel_integrator_torque;
    }

    // 力矩模式下，速度限制
    if (Motor::ControlMode::kVelocity > m_motor.getControlMode()
                && cfg_motor.torque_mode_vel_limit_enabled) {
        if (!std::isfinite(vel_est)) {
            // TODO: error
            return false;
        }

        torque = limitVelocity(vel_est, vel_gain, cfg_motor.vel_limit, torque);
    }

    // Torque limiting
    bool torque_limited = false;
    if (torque > Tlim) {
        torque_limited = true;
        torque = Tlim;
    } else if (torque < -Tlim) {
        torque_limited = true;
        torque = -Tlim;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (Motor::ControlMode::kVelocity > m_motor.getControlMode()) {
        m_vel_integrator_torque = 0.f;

    } else {
        if (torque_limited) {
            m_vel_integrator_torque *= 0.99f;

        } else {
            m_vel_integrator_torque +=
                ((vel_integrator_gain * gain_scheduling_multiplier)
                    * PICA_DRIVE_CURRENT_MEASURE_PERIOD)
                * vel_err;
        }

        m_vel_integrator_torque = std::clamp(m_vel_integrator_torque,
            -m_cfg.vel_integrator_limit,
             m_cfg.vel_integrator_limit);
    }

    *torque_ref = torque;

    return true;
}
