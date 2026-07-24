/**
 * @file current_controller_phase_inductance_measurer.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-14
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_BLDC_PHASE_INDUCTANCE_MEASURER_HPP_
#define _PICA_DRIVE_BLDC_PHASE_INDUCTANCE_MEASURER_HPP_

#include "current_controller.hpp"
#include "hrt.h"

namespace pica
{

namespace motor
{

namespace bldc
{

class PhaseInductanceMeasurer : public CurrentController<PhaseInductanceMeasurer>
{
    friend class CurrentControllerVariant;
    friend class CurrentControllerProxy;
    friend CurrentController<PhaseInductanceMeasurer>;

public:

    float inductance() const
    {
        // Note: A more correct formula would also take into account that there is a finite timestep.
        // However, the discretisation in the current control loop inverts the same discrepancy
        float dt = ((float)hrt_diff_nano(&m_timestamp_last, &m_timestamp_start) * 1e-9f);

        return std::abs(m_voltage_test) / (m_Idelta / dt);
    }

    void reset()
    {
        m_attached = false;
    }

    bool init(void *cfg) { return true; }

    /**
     * @brief 计算相电阻
     * 
     * @param[in] period 
     * @return true 
     * @return false 
     */
    bool update(float torque_sp, hrt_absnano ts_output);

    /**
     * @brief 更新输出(alpha-beta)
     * 
     * @param[in] time2last_meas 
     * @param[in] time2next_pwm_output 
     * @param[in] period 
     * @return true 
     * @return false 
     */
    bool run(hrt_absnano ts_output, AlphaBeta *v_alpha_beta_final);

private:
    bool m_attached{false};

    float m_voltage_test = 0.f;
    float m_sign = 0.f;
    float m_Ialpha_prev = NAN;
    float m_Idelta = 0.f;

    hrt_absnano m_timestamp_start = 0;
    hrt_absnano m_timestamp_last = 0;
};
    
} // namespace bldc

    
} // namespace motor

    
} // namespace pica


#endif /* !_PICA_DRIVE_BLDC_PHASE_INDUCTANCE_MEASURER_HPP_ */
