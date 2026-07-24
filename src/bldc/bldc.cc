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

#include "bldc/bldc.hpp"
#include "utils/math.h"
#include "config_manager.hpp"

#include <cmath>
#include <algorithm>

#if (PICA_DRIVE_ENABLE_LOGGER == 1)

#include "spdlog/spdlog.h"

#endif

using namespace pica::motor::bldc;

bool BLDC::init()
{
    auto cfg_mgr = ConfigMgr::GetInstance();

    m_cfg = *reinterpret_cast<Config*>(cfg_mgr->motor());
    if (m_cfg.motor_type < 0) {
        #if (PICA_DRIVE_ENABLE_LOGGER == 1)
                spdlog::info("BLDC: motor type {} is invalid.", m_cfg.motor_type);
        #endif
    }

    #if (PICA_DRIVE_ENABLE_LOGGER == 1)
        spdlog::info("BLDC: ConfigMgr addr at {}, motor at {}, type {}, current controller {}, speed controller {}, control mode {}",
                (uint64_t)ConfigMgr().GetInstance(),
                (uint64_t)(ConfigMgr().GetInstance()->motor()),
                this->type(),
                m_cfg.current_controller_type,
                m_cfg.speed_controller_type,
                m_cfg.motor_control_mode
        );

        spdlog::info("BLDC: addr of foc({}), addr of ctx in proxy({})", 
            (uint64_t)&m_current_controller, 
            (uint64_t)m_current_controller_proxy.getInstance<FOC>()
        );
    #endif
    
    m_pole_pairs = m_cfg.pole_pairs;

    if (!m_current_controller_proxy.init(cfg_mgr->current())) {
        #if (PICA_DRIVE_ENABLE_LOGGER == 1)
            spdlog::info("BLDC: init current controller failed.");
        #endif

        return false;
    }

    calcPhaseCurrentGain();

    // 初始化速度控制器
    m_speed_controller.emplace(static_cast<speed::Type>(m_cfg.speed_controller_type), *this);
    
    if (!m_speed_controller.init(cfg_mgr->speed())) {
        #if (PICA_DRIVE_ENABLE_LOGGER == 1)
                spdlog::info("BLDC: init speed controller failed.");
        #endif

        return false;
    }

    return true;
}

bool BLDC::update(hrt_absnano now)
{
    float torque = 0.f;

    if (!m_speed_controller.update(now, &torque)) {
        return false;
    }

    if (!m_current_controller_proxy.update(torque, now)) {
        #if (PICA_DRIVE_ENABLE_LOGGER == 1)
                spdlog::info("BLDC: update current controller failed.");
        #endif

        return false;
    }

    return true;
}

float BLDC::calcEffectiveCurrentLimit()
{
    float current_limit = m_cfg.current_limit; // Configured limit

	// Hardware limit
	if (BLDC::kGimbal == type()) {
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

float BLDC::calcMaxAvailableTorque()
{
    float torque;

    if (BLDC::Type::kACIM == type()) {
        // torque = motor->effective_current_limit * param->torque_constant * axis_->acim_estimator_.rotor_flux_;

    } else {
        torque = m_effective_current_limit * m_cfg.torque_constant;
    }

    torque = std::clamp(torque, 0.f, m_cfg.torque_limit);

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
            k_margin * max_output_swing * m_cfg.shunt_conductance;
    // 需用增益 = 最大可测电流 / 最大电流
    float requested_gain = max_unity_gain_current / m_cfg.requested_current_range;

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

bool BLDC::measurePhaseResistance(float test_current, float max_voltage)
{
    PhaseResistanceMeasurer measurer{*this, test_current, max_voltage};

    m_current_controller_proxy.reset(&measurer);

    arm(&measurer);

    return true;
}

bool BLDC::disarm()
{

    return true;
}