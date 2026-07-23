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

#ifndef _PICA_DRIVE_AXIS_CONTROLLER_PI_HPP_
#define _PICA_DRIVE_AXIS_CONTROLLER_PI_HPP_

#include "motor/speed_controller.hpp"

namespace pica
{

namespace motor
{

struct SpeedControllerConfig;

class SpeedControllerPI : public SpeedController
{
public:
    SpeedControllerPI(Motor& motor) : SpeedController(motor) {}

    /**
     * @brief 使用 Motor 输入更新输出力矩/电流指令
     * 
     * @param[in] ts2last_running 
     * @return true 
     * @return false 
     */
    bool update(hrt_absnano now, float *torque) final;

    void reset()
    {
        m_integrator_torque = 0.f;
    }

private:

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
    
} // namespace axis


    
} // namespace axis


#endif /* !_PICA_DRIVE_AXIS_CONTROLLER_PI_HPP_ */
