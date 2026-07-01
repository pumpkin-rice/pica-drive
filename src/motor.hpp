/**
 * @file motor.h
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_MOTOR_H_
#define _PICA_MOTOR_H_

#include "motor/config.h"
#include "motor/current_controller.hpp"
#include "utils/noncopyable.hpp"
#include <stdint.h>

namespace pica
{

namespace motor
{
    
// class CurrentController;

} // namespace motor

/**
 * @brief 电机输入参数为力矩,由电流控制器生成控制电压,实现 Axis 与 Motor 解耦
 * 
 */
class Motor : Noncopyable
{
public:
    using CurrentController = motor::CurrentController;

    virtual bool init(motor_config *cfg)
    {
        if (cfg) {
            m_cfg = cfg;
        }

        return true;
    }

    virtual bool update() = 0;

    virtual void sampleCurrentHandler(const float shunt_volt[]) {}

    virtual void sampleCurrentCalibratorHandler(const float *calibator, float measure_period) {}

    void sampleBusVoltageHandler(const float voltage)
    {
        m_bus_voltage_meas = voltage;
    }

    void sampleBusCurrentHandler(const float curr)
    {
        m_bus_current_meas = curr;
    }

    void sampleEncoderHandler(float angle_mach, float angular_velocity_mach)
    {
        float np = m_cfg->pole_pairs;
        m_angle_elec = angle_mach * np;
        m_angular_velocity_elec = angular_velocity_mach * np;
    }

    /**
     * @brief 运行控制器
     * 
     * @param[in] theta_mach 
     * @param[in] omega_mach 
     * @param[in] time2last_meas 
     * @param[in] time2next_pwm_output 
     * @return true 
     * @return false 
     */
    bool runControllerLoop(float time2last_meas, float time2next_pwm_output,
                    float period)
    {
        return m_controller_loop_func(m_current_controller,
                    time2last_meas, time2next_pwm_output, period
                );
    }

    /**
     * @brief Get the Duty Cycles object
     * 
     * @return const float* 
     */
    virtual const float *getDutyCycles() = 0;

    virtual bool do_checks() { return true; }

    constexpr uint8_t getMotorType() { return m_cfg->motor_type; }

    constexpr float electricalAngle() { return m_angle_elec; }
    constexpr float electricalAngualrVelocity() { return m_angular_velocity_elec; }

    virtual float getMaxAvailableTorque() = 0;

    virtual float getEffectiveCurrentLimit() { return m_effective_current_limit; }

    float setTorque(float t)
    {
        return m_torque_sp = t;
    }

protected:
    void setControllerLoopFunction(CurrentController::ControllerLoopFuncType func,
                                    CurrentController *ctrl)
    {
        m_controller_loop_func = func;
        m_current_controller = ctrl;
    }

protected:
    motor_config *m_cfg;

    CurrentController *m_current_controller{nullptr};
    /**
     * @brief 控制器程序
     * 
     */
    CurrentController::ControllerLoopFuncType m_controller_loop_func{nullptr};

    float m_bus_voltage_meas; /*!< 母线电压, V */
    float m_bus_current_meas; /*!< 母线电流, A */

    float m_torque_sp; /*!< Nm */
    float m_effective_current_limit; /*!< A */
    float m_max_allowed_current; /*!< A */
    float m_max_dc_calib;
    float m_phase_current_rev_gain; /*!< ADC 输出电压与 Shunt 的比例 */
    float m_power_watt;

    float m_angle_elec;
    float m_angular_velocity_elec;
};

} // namespace pica


typedef struct __motor_object motor_obj;
typedef struct __current_controller_object current_controller_obj;

#define __STRUCT_MOTOR_INF \
struct { \
    int (*update)(motor_obj *motor); /*!< 使用输入数据更新内部状态 */ \
    void (*sample_current_handler)(motor_obj *motor, const float *shunt_volt); /*!< 电流采样更新 */ \
    void (*sample_current_calibrator_handler)(motor_obj *motor, const float calib[], float measure_period); \
    void (*sample_voltage_handler)(motor_obj *motor, const float *voltage_phases); \
    void (*get_duty_cycles)(motor_obj *motor, float *duty_cycle); \
    int (*update_config)(motor_obj *motor); \
    float (*get_max_available_torque)(motor_obj *motor); \
    int (*do_checks)(motor_obj *motor); \
    struct motor_parameters *param; \
    current_controller_obj *current_controller; \
}

#define __STRUCT_MOTOR_DATA \
struct { \
    struct motor_parameters *param; \
    float theta_elec; /*!< electrical angle, rad */ \
    float omega_elec; /*!< electrical angular velocity, rad/s */ \
    float torque_sp; /*!< Nm */ \
    float effective_current_limit; /*!< A */ \
    float max_allowed_current; /*!< A */ \
    float max_dc_calib; \
    float phase_current_rev_gain; /*!< ADC 输出电压与 Shunt 的比例 */ \
    float bus_voltage_meas; \
    float bus_current_meas; \
    float power_watt; \
    uint32_t sampling_timestamp_ns; /*!< timer tick */ \
    bool direction; /*!< direction of rotator: true for anticlockwise, false for clockwise */ \
}

/**
 * @brief 定义具体电机类型的时候需要在最开始包含此 macro
 * 
 */
#define MOTOR_OBJECT \
struct { \
    /* __STRUCT_MOTOR_INF; */ \
    __STRUCT_MOTOR_DATA; \
}

struct __motor_object
{
    MOTOR_OBJECT;
};

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif

#endif /* !_PICA_MOTOR_H_ */
