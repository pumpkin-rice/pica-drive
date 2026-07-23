/**
 * @file speed_controller.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-02
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_MOTOR_SPEED_CONTROLLER_HPP_
#define _PICA_DRIVE_MOTOR_SPEED_CONTROLLER_HPP_

#include "motor/config.hpp"
#include "utils/noncopyable.hpp"
#include <stdint.h>

namespace pica
{

class Motor;

namespace motor
{

class SpeedControllerConfig;

/**
 * @brief 速度控制器，控制电机位置、速度
 * 
 */
class SpeedController
{
public:
    enum ControlMode : int8_t
    {
        kTorque = 0,
        kVelocity,
        kPosition,
    };

    SpeedController(Motor& motor) : m_motor(motor) {}

    virtual bool init(const SpeedControllerConfig *cfg)
    {
        m_cfg = cfg;

        return true;
    }

    /**
     * @brief 
     * 
     * @param[in] now 当前时刻，ns
     * @param[in] torque 输出力矩
     * @return true 
     * @return false 
     */
    virtual bool update(hrt_absnano now, float *torque)
    {
        *torque = 0.2f;
        return true;
    };

    virtual void reset()
    {
    }

protected:
   

protected:
    Motor& m_motor;
    const SpeedControllerConfig *m_cfg{nullptr};

    hrt_absnano m_ts_prev{0};
};

} // namespace motor


} // namespace pica


#endif /* !_PICA_DRIVE_MOTOR_SPEED_CONTROLLER_HPP_ */
