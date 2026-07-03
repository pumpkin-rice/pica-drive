/**
 * @file bldc.cc
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "motor/bldc.hpp"
#include "utils/math.h"
#include <cmath>
#include <algorithm>

using namespace pica::motor;

bool BLDC::init(Config *cfg)
{
    if (!Motor::init(cfg)) {
        return false;
    }

    switch (cfg->current_controller_type) {
    case CurrentControllerType::FieldOrientedControl:
    {
        m_current_controller =
                &m_current_controller_variant.emplace<FOC>(*this);
        setControllerLoopFunction(
            m_current_controller->getControllerLoopFunc(),
            m_current_controller
        );
        break;
    }

    default:
        return false;
    }

    m_current_controller->init(cfg);

    calcPhaseCurrentGain();

    // // 初始化速度控制器
    // m_speed_controller = 
    //         &m_speed_controller_variant.emplace<SpeedControllerPI>(*this);
    // m_speed_controller->init(&m_cfg->speed_controller_cfg);

    return true;
}

float BLDC::getEffectiveCurrentLimit()
{
    float current_limit = m_cfg->current_limit; // Configured limit

	// Hardware limit
	if (BLDC::GIMBAL == m_cfg->motor_type) {
		current_limit = fminf(current_limit, 0.98f * FRAC_1_SQRT3 * m_bus_voltage_meas); //gimbal motor is voltage control
	
	} else {
		current_limit = fminf(current_limit, 
							m_max_allowed_current);
	}

	// TODO: thermistor
	// Apply thermistor current limiters
    // current_lim = std::min(current_lim, motor_thermistor_.get_current_limit(config_.current_lim));
    // current_lim = std::min(current_lim, fet_thermistor_.get_current_limit(config_.current_lim));

	m_effective_current_limit = current_limit;

	return current_limit;
}

float BLDC::getMaxAvailableTorque()
{
    float torque;

    if (BLDC::ACIM == m_cfg->motor_type) {
        // torque = motor->effective_current_limit * param->torque_constant * axis_->acim_estimator_.rotor_flux_;

    } else {
        torque = m_effective_current_limit * m_cfg->torque_constant;
    }

    torque = std::clamp(torque, 0.f, m_cfg->torque_limit);

    return torque;
}

void BLDC::calcPhaseCurrentGain()
{
    // TODO: 电流限制传感器
    // fet_thermistor_.update();
    // motor_thermistor_.update();

    // Solve for exact gain, then snap down to have equal or larger range as requested
    // or largest possible range otherwise
    const float k_margin = 0.9f;
    const float max_output_swing = 1.35f; /*!< out of amplifier, V */

    // 可测量最大电流 = 限幅 * ADC输出范围 * 电导
    float max_unity_gain_current =
            k_margin * max_output_swing * m_cfg->shunt_conductance;
    // 需用增益 = 最大可测电流 / 最大电流
    float requested_gain = max_unity_gain_current / m_cfg->requested_current_range;

    float actual_gain; /*!< 实际增益 */
    if (0) {
    // if (!gate_driver_.config(requested_gain, &actual_gain))
    //      return false;
    } else {
        actual_gain = requested_gain;
    }

    // 实际电压： adc 测得电压* motor->phase_current_rev_gain
    // 实际电流： 实际电压 / 电阻 
    m_phase_current_rev_gain = 1.f/actual_gain;
    m_max_allowed_current = max_unity_gain_current
            * m_phase_current_rev_gain;

    m_max_dc_calib = 0.1f * m_max_allowed_current;
}
