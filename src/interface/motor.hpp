/**
 * @file motor.hpp
 * @author Pumpkin Rice
 * @brief 电机接口定义
 * @version 0.1
 * @date 2026-07-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_MOTOR_HPP_
#define _PICA_DRIVE_MOTOR_HPP_

#include "hrt.h"
#include "utils/noncopyable.hpp"
#include <stdint.h>

namespace pica
{

/**
 * @brief 电机基类：声明电机公共接口
 * 
 * 电机类别与实际PCB之间为绑定关系，所以没有使用 CRTP——直接调用子类的接口效率是一样的
 * 
 */
class Motor : Noncopyable
{
public:
    /**
     * @brief 电机基本类型（大类），子类单独定义
     * 
     * 使用 [7:15] 作为电机基本类型
     */
    enum BasicType : int16_t
    {
        kUnknow        = -1,
        kBasicTypeDC   = 0,
        kBasicTypeBLDC,

        _kBasicTypeMax,
    };

    /**
     * @brief 电机控制模式
     * 
     */
    enum class ControlMode : int8_t
    {
        kUnknow = -1,
        kTorque = 0, /*!< 力矩控制 */
        kVelocity, /*!< 速度控制 */
        kPosition, /*!< 位置控制 */

        _kMax,
    };

    constexpr BasicType GetMotorBaiscType(int16_t type)
    {
        type >>= 8;

        if (type > _kBasicTypeMax || type < 0) {
            return kUnknow;
        }

        return static_cast<BasicType>(type);
    }

    virtual bool init() = 0;

    virtual bool update(hrt_absnano ts) = 0;

    /**
     * @brief 运行电机控制环程序
     * 
     * @param[in] ts_output 距离下一次pwm更新时间，tick
     * @return true 
     * @return false 
     */
    virtual bool run(hrt_absnano ts_output) = 0;

    /**
     * @brief 获取 PWM 占空比：[0..1]
     * 
     * @return const float* 
     */
    virtual const float *dutyCycle() = 0;

    virtual bool do_checks() = 0;

    virtual bool arm(void *current_controller) = 0;
    virtual bool disarm() = 0;
    bool isArmed() const { return m_armed; };

    /**
     * @brief 电流采样更新
     * 
     * @param[in] shunt_volt 
     * @param[in] ts 采样时间，tick
     */
    virtual void sampleCurrentHandler(const float shunt_volt[], hrt_absnano ts) = 0;
    /**
     * @brief 校正电流采样更新
     * 
     * @param[in] shunt_volt 
     * @param[in] period 采样周期，sec
     */
    virtual void sampleCurrentCalibratorHandler(const float shunt_volt[], float period) = 0;

    /**
     * @brief 母线电流采样
     * 
     * @param[in] voltage 
     */
    void sampleBusVoltageHandler(float voltage)
    {
        m_bus_voltage_meas = voltage;
    }

    /**
     * @brief 更新编码器数据
     * 
     * @note 对于电角度与编码器采样角度不一致时，需要重写此接口
     * 
     * @param[in] pos rad
     * @param[in] vel rad/s
     */
    void sampleEncoderHandler(float pos, float vel)
    {
        m_position_est = pos * m_pole_pairs;
        m_velocity_est = vel * m_pole_pairs;
    }

    /**
     * @brief 获取电角度
     * 
     * @return float 
     */
    float positionEst() const { return m_position_est; }
    /**
     * @brief 获取电角速度
     * 
     * @return * float 
     */
    float velocityEst() const { return m_velocity_est; }

    int polePairs() const { return m_pole_pairs; }

    float busVoltage() const { return m_bus_voltage_meas; }

    virtual float calcMaxAvailableTorque() = 0;

    float effectiveCurrentLimit() const
    {
        return m_effective_current_limit;
    }

    virtual ControlMode controlMode() const = 0;

    const hrt_absnano& timestampCurrentMeas() const { return m_ts_current_meas; }

    void setTorqueSetpoint(float t) { m_torque_sp = t; }
    float torqueSetpoint() const { return m_torque_sp; }

    /**
     * @brief Set the machincial Velocity object
     * 
     * @param[in] vel rad/s
     */
    void  setVelocitySetpoint(float vel) { m_velocity_sp = vel * m_pole_pairs; }
    float velocitySetpoint() const { return m_velocity_sp; }
    /**
     * @brief Set the Position object
     * 
     * @param[in] pos rad
     */
    void  setPositionSetpoint(float pos) { m_position_sp = pos * m_pole_pairs; }
    float positionSetpoint() const { return m_position_sp; }

protected:
    bool m_armed{false};

    float m_pole_pairs{1.f};

    float m_position_sp{0.f}; /*!< electrical angle, rad */
    float m_velocity_sp{0.f}; /*!< electrical angular velocity, rad/s */
    float m_torque_sp{0.f};   /*!< Nm */

    float m_position_est{NAN}; /*!< electrical angle, rad */
    float m_velocity_est{NAN}; /*!< electrical angular velocity, rad/s */

    float m_bus_voltage_meas; /*!< V */
    float m_bus_current_meas; /*!< A */
    float m_power_watt; /*!< 当前功耗，W */

    float m_effective_current_limit{10.f}; /*!< 电流限制数值，A */

    hrt_absnano m_ts_current_meas{0}; /*!< 电流采样时刻, tick */
};

    
} // namespace pica


#endif /* !_PICA_DRIVE_MOTOR_HPP_ */
