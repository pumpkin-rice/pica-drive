/**
 * @file openloop_controller.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_BLDC_OPENLOOP_CONTROLLER_HPP_
#define _PICA_DRIVE_BLDC_OPENLOOP_CONTROLLER_HPP_

#include "bldc/park.h"
#include "hrt.h"
#include <algorithm>

namespace pica
{

namespace motor
{

namespace bldc
{

class BLDCOpenloopController
{
public:
    void update(hrt_abstick tick)
    {
        float phase = std::isfinite(m_phase) ? m_phase : m_initial_pahse;
        float phase_vel = std::isfinite(m_phase_vel) ? m_phase_vel : 0.f;

        DQ idq_prev = m_idq_sp;
        DQ vdq_prev = m_vdq_sp;

        float delta_time = hrt_tick_to_second(tick - m_tick_prev);

        m_idq_sp = {
            std::clamp(m_current_sp, 
                        idq_prev(0) - m_max_current_ramp * delta_time, 
                        idq_prev(0) + m_max_current_ramp * delta_time),
            0.f,
        };
        m_vdq_sp = {
            std::clamp(m_voltage_sp,
                        vdq_prev(0) - m_max_voltage_ramp * delta_time,
                        vdq_prev(0) + m_max_voltage_ramp * delta_time),
            0.f
        };

        phase_vel = std::clamp(m_velocity_sp,
                            phase_vel - m_max_phase_vel_ramp * delta_time,
                            m_phase_vel + m_max_phase_vel_ramp * delta_time);
        m_phase_vel = phase_vel;

        m_phase = wrap_pm_pi(phase + phase_vel * delta_time);
        m_total_distance = m_total_distance + phase_vel * delta_time;

        m_tick_prev = tick;
    }

private:
    hrt_abstick m_tick_prev;

    DQ m_idq_sp{0.f, 0.f};
    DQ m_vdq_sp{0.f, 0.f};

    float m_max_current_ramp{INFINITY};
    float m_max_voltage_ramp{INFINITY};
    float m_max_phase_vel_ramp{INFINITY};

    float m_velocity_sp{0.f}; /*!< elec, rad/s */
    float m_current_sp{0.f};
    float m_voltage_sp{0.f};
    float m_initial_pahse{0.f};

    float m_phase{0.f};
    float m_phase_vel{0.f};
    float m_total_distance{0.f};
};

    
} // namespace bldc

} // namespace motor

    
} // namespace pica


#endif /* !_PICA_DRIVE_OPENLOOP_CONTROLLER_HPP_ */
