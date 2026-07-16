/**
 * @file current_controller_phase_inductance_measurer.cc
 * @author Pumpkin Rice
 * @brief
 * @version 0.1
 * @date 2026-07-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "phase_inductance_measurer.hpp"

#include "bldc/bldc.hpp"

using namespace pica::motor::bldc;

bool PhaseInductanceMeasurer::update(hrt_absnano now) {
  AlphaBeta Ialpha_beta = clarke(m_motor.curentMeasured());

  if (!isfinite(Ialpha_beta)) {
    // TODO:

    return false;
  }

  float Ialpha = Ialpha_beta(0);

  if (m_attached) {
    float sign = sign_hard(m_voltage_test);
    m_Idelta += -sign * (Ialpha - m_Ialpha_prev);

  } else {
    m_timestamp_start = now;
    m_attached = true;
  }

  m_timestamp_last = now;

  return false;
}

bool PhaseInductanceMeasurer::run(hrt_absnano ts_output,
                                  AlphaBeta* v_alpha_beta_final) {
  m_voltage_test *= -1.f;
  float vfactor = 1.f / ((2.f / 3.f) * m_motor.busVoltage());

  *v_alpha_beta_final = {m_voltage_test * vfactor, 0.f};

  return true;
}
