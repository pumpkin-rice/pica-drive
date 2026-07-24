/**
 * @file PI.hpp
 * @author Pumpkin Rice
 * @brief 基于 PI 控制器的轴控制器
 * @version 0.1
 * @date 2026-06-27
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_BLDC_CONTROLLER_PI_HPP_
#define _PICA_DRIVE_BLDC_CONTROLLER_PI_HPP_

#include "speed_controller.hpp"
#include <cmath>

namespace pica
{

namespace motor
{

namespace bldc
{

class BLDC;

class SpeedControllerPI : public SpeedController<SpeedControllerPI>
{
    friend SpeedController<SpeedControllerPI>;
    
public:
    struct Config
    {
        float pos_gain = 20.f; /*!< (turn/s)/turn */
        float vel_gain = 1.f/6.f; /*!< Nm/(turn/s) */
        float vel_integrator_gain = 2.0f/6.0f;
        float vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.

        float gain_scheduling_width = 10.0f;
        bool gain_scheduling_enabled = true;
    };

    SpeedControllerPI(BLDC& motor) : SpeedController(motor) {}

    bool init(void *cfg);

    /**
     * @brief 运行速度环控制器，更新参考力矩/电流指令
     * 
     * @param[in] torque_ref 参考力矩输出
     * @param[in] now 
     * @return true 
     * @return false 
     */
    bool update(hrt_absnano now, float *torque_ref);

    void reset()
    {
        m_integrator_torque = 0.f;
    }

private:
    Config m_cfg;

    float m_integrator_torque = 0.f;
    float m_vel_integrator_torque = 0.f;

    float m_input_pos = 0.0f;     // [turns]
    float m_input_vel = 0.0f;     // [turn/s]
    float m_input_torque = 0.0f;  // [Nm]
    float m_input_filter_kp = 0.0f;
    float m_input_filter_ki = 0.0f;

    bool m_anticogging_valid = false;
    float m_mechanical_power = 0.0f; // [W]
    float m_electrical_power = 0.0f; // [W]
};

} // namespace bldc
    
} // namespace axis
    
} // namespace axis


#endif /* !_PICA_DRIVE_AXIS_CONTROLLER_PI_HPP_ */
