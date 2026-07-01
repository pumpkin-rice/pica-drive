/**
 * @file bldc.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_MOTOR_BLDC_HPP_
#define _PICA_DRIVE_MOTOR_BLDC_HPP_

#include "motor.hpp"
#include "motor/park/park.h"
#include "bldc/foc.hpp"

#include <variant>

namespace pica {

namespace motor
{

class FOC;

class BLDC : public Motor
{
    friend CurrentController;
    friend FOC;

public:
    enum Type : uint8_t
    {
        ACIM = 0, /*!< 异步感应 */
        GIMBAL, /*!< 云台 */
        HIGH_CURRENT, /*!< 大电流 */
    };

    enum CurrentControllerType : uint8_t
    {
        FieldOrientedControl = 0,
    };

    bool init(motor_config *cfg) final;

    bool update() override
    {
        return m_current_controller->update();
    }

    void sampleCurrentHandler(const float shunt_volt[]) final
    {
        float shunt_gain = m_cfg->shunt_conductance * m_phase_current_rev_gain;

        m_current.set(
            shunt_gain * shunt_volt[0],
            shunt_gain * shunt_volt[1],
            shunt_gain * shunt_volt[2]
        );
    }

    void sampleCurrentCalibratorHandler(const float *calibator, float measure_period) final
    {
        float *c = m_current_calibrator.raw;
        
        if (NULL == calibator) {
            c[0] = 0.f;
            c[1] = 0.f;
            c[2] = 0.f;

            m_current_calibrator_running_since = 0.f;

        } else {
            const float k_filter = std::min(
                measure_period/m_cfg->dc_current_calibrator_filter_tau,
                1.f
            );

            c[0] += (calibator[0] - c[0]) * k_filter;
            c[1] += (calibator[1] - c[1]) * k_filter;
            c[2] += (calibator[2] - c[2]) * k_filter;

            m_current_calibrator_running_since += measure_period;
        }
    }

    float getEffectiveCurrentLimit() final;

    float getMaxAvailableTorque() final;

    const float *getDutyCycles() final { return m_duty_cycle.raw; }

private:
    void calcPhaseCurrentGain();

private:
    ThreePhase m_current;
    ThreePhase m_current_calibrator;
    ThreePhase m_duty_cycle;

    float m_current_calibrator_running_since; // current sensor calibration needs some time to settle

    std::variant<FOC> m_current_controller_variant{*this};
};

}
}

#endif /* !_PICA_DRIVE_MOTOR_BLDC_HPP_ */
