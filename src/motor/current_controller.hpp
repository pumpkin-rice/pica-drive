/**
 * @file current_controller.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_MOTOR_CURRENT_CONTROLLER_HPP_
#define _PICA_DRIVE_MOTOR_CURRENT_CONTROLLER_HPP_

#include "utils/noncopyable.hpp"

namespace pica
{
class Motor;

namespace motor
{

class Config;

class CurrentController : Noncopyable
{
public:

    typedef bool (*ControllerLoopFuncType)(CurrentController *ctrl,
                float time2last_meas, float time2next_pwm_output,
                float period);

    CurrentController(Motor& motor) : m_motor(motor)
    {
    }

    virtual bool init(const Config *cfg)
    {
        m_cfg = cfg;

        return true;
    }

    /**
     * @brief 更新控制参数
     * 
     * 1. 根据力矩生成电流设定值
     * 
     * @param[in] torque 参考力矩, Nm
     * @return true 
     * @return false 
     */
    virtual bool update() { return true; };

    virtual void updateGain() {};
    virtual void reset() {};

    virtual CurrentController::ControllerLoopFuncType getControllerLoopFunc() = 0;

protected:

    Motor& m_motor;

    const Config *m_cfg;

    bool m_current_loop_enabled = false; /*!< 是否使用电流环: true - 使用电流环生成控制电压; false - 使用电压环*/
};

} // namespace pica
}

#endif /* !_PICA_DRIVE_CURRENT_CONTROLLER_HPP_ */
