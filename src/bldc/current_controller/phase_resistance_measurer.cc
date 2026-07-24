/**
 * @file current_controller_phase_resistance_measurer.cc
 * @author Pumpkin Rice
 * @brief
 * @version 0.1
 * @date 2026-07-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "phase_resistance_measurer.hpp"

#include <algorithm>

#include "bldc/bldc.hpp"

namespace pica {

namespace motor {

namespace bldc {

bool PhaseResistanceMeasurer::update(float torque_sp, hrt_absnano now)
{
    auto& curr = m_motor.currentMeasured();
    auto vbus = m_motor.busVoltage();
    auto Ialpha_beta = clarke(curr);

    if (isfinite(Ialpha_beta)) {
        m_current_actual = Ialpha_beta(0);

        m_voltage_test +=
            m_ki * PICA_DRIVE_CURRENT_MEASURE_PERIOD * (m_current_target - m_current_actual);
        m_Ibeta +=
            (m_k_Ibeta_filter * PICA_DRIVE_CURRENT_MEASURE_PERIOD) * (Ialpha_beta(1) - m_Ibeta);

    } else {
        m_current_actual = 0.f;
        m_voltage_test = 0.f;
    }

    if (std::abs(m_voltage_test) > m_voltage_max) {
        m_voltage_test = NAN;
        return false;

    } else if (std::isfinite(vbus)) {
        float vfactor = 1.f / ((2.f / 3.f) * vbus);
        m_test_mod = m_voltage_test * vfactor;

        return true;
    }

    return false;
}

bool PhaseResistanceMeasurer::run(hrt_absnano ts_output,
                                  AlphaBeta* v_alpha_beta_final) {
    if (std::isfinite(m_test_mod)) {
        // TODO: error noto inited
        return false;
    }

    m_motor.m_bus_current_meas = m_test_mod * m_current_actual;

    // 使用该结构临时存储输出
    *v_alpha_beta_final = {m_test_mod, 0.f};

    return true;
}

}  // namespace bldc

}  // namespace motor

}  // namespace pica
