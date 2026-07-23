/**
 * @file current_controller_phase_resistance_measurer.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-14
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_PHASE_RESISTANCE_MEASURER_HPP_
#define _PICA_DRIVE_PHASE_RESISTANCE_MEASURER_HPP_

#include "current_controller.hpp"

namespace pica
{

namespace motor
{

namespace bldc
{

class PhaseResistanceMeasurer : public CurrentController<PhaseResistanceMeasurer>
{
    friend class CurrentControllerVariant;
    friend class CurrentControllerRefVariant;
    friend class CurrentController<PhaseResistanceMeasurer>;

public:
    PhaseResistanceMeasurer(BLDC& motor, float test_current, float max_voltage)
        : m_voltage_max(max_voltage)
        , m_current_target(test_current)
        , CurrentController(motor)
        {}

    /**
     * @brief 获取 beta 电流
     * 
     * @return float 
     */
    float Ibeta() const { return m_Ibeta; }

    /**
     * @brief 获取相电阻
     * 
     * @return float 
     */
    float resistance() const { return m_voltage_test / m_current_target; }

    void reset()
    {
        m_voltage_test = 0.f;
        m_test_mod = NAN;
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
     * @brief 计算下一次输出参c
     * 
     * @param[in] time2last_meas 
     * @param[in] time2next_pwm_output 
     * @param[in] period 
     * @return true 
     * @return false 
     */
    bool run(hrt_absnano ts_output, AlphaBeta *v_alpha_beta_final);

private:
    const float m_ki = 1.f; /*!< (V/s)/A */
    const float m_k_Ibeta_filter = 80.f;

    float m_voltage_max = 0.f;
    float m_voltage_test = 0.f;

    float m_current_actual = 0.f;
    float m_current_target = 0.f;

    float m_Ibeta = 0.f; /*!< low pass filtered I_beta response */
    float m_test_mod = NAN;
};

    
} // namespace bldc

    
} // namespace motor

    
} // namespace pica


#endif /* !_PICA_DRIVE_PHASE_RESISTANCE_MEASURER_HPP_ */
