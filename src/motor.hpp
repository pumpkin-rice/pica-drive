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

#include "motor/config.hpp"
#include "motor/current_controller.hpp"
#include "motor/speed_controller.hpp"
#include "utils/noncopyable.hpp"
#include "utils/math.h"
#include "hrt.h"
#include <stdint.h>

namespace pica
{

namespace motor
{
    
class SpeedController;
class CurrentController;

} // namespace motor

/**
 * @brief 电机输入参数为力矩,由电流控制器生成控制电压,实现 Axis 与 Motor 解耦
 * 
 */
class Motor : Noncopyable
{
    friend motor::CurrentController;

public:
    using SpeedController = motor::SpeedController;
    using CurrentController = motor::CurrentController;
    using Config = motor::Config;

    virtual bool init(Config *cfg)
    {
        if (cfg) {
            m_cfg = cfg;
        }

        return true;
    }

    /**
     * @brief
     * 
     * @param[in] period update 运行周期 s
     * @return true 
     * @return false 
     */
    virtual bool update(hrt_absnano now) = 0;

    /**
     * @brief 运行控制器
     * 
     * @param[in] time2meas 到电流、角度测量时间, s
     * @param[in] time2pwm_output 当前时刻到下次 PWM 输出时间, s
     * @return true 
     * @return false 
     */
    bool run(hrt_absnano ts_next_pwmoutput)
    {
        return m_controller_loop_func(m_current_controller, ts_next_pwmoutput);
    }

    /**
     * @brief 电流采样回调
     * 
     * @param[in] shunt_volt 
     * @param[in] meas_ts 采样时间，ns
     */
    virtual void sampleCurrentHandler(const float shunt_volt[], hrt_absnano meas_ts) {}

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
        m_position_est = angle_mach * np;
        m_velocity_est = angular_velocity_mach * np;
    }

    /**
     * @brief Get the Duty Cycles object
     * 
     * @return const float* 
     */
    virtual const float *getDutyCycles() = 0;

    virtual bool do_checks() { return true; }

    int8_t getMotorType() const { return m_cfg->motor_type; }
    int8_t getCurrentControllerType() const { return m_cfg->current_controller_type; }

    virtual float getMaxAvailableTorque() = 0;

    virtual float getEffectiveCurrentLimit() { return m_effective_current_limit; }

    constexpr float getPhaseCurrentRevGain() { return m_phase_current_rev_gain; }

    float getElectricalPositionEst() const { return m_position_est; }
    float getElectricalVelocityEst() const { return m_velocity_est; }
    float getElectricalPositionSetpoint() const { return m_position_sp; }
    float getElectricalVelocitySetpoint() const { return m_velocity_sp; }

    float setPosition(float pos) { return m_position_sp = pos * m_cfg->pole_pairs; }
    float setVelocity(float vel) { return m_velocity_sp = vel * m_cfg->pole_pairs; }
    float setTorque(float t) { return m_torque_sp = t; }
    float getTorqueSetpoint() const { return m_torque_sp; }

    const hrt_absnano& currentSampleTimestamp() const { return m_current_sample_ts; }

protected:
    void setControllerLoopFunction(CurrentController::ControllerLoopFuncType func,
                                    CurrentController *ctrl)
    {
        m_controller_loop_func = func;
        m_current_controller = ctrl;
    }

protected:
    Config *m_cfg;

    CurrentController *m_current_controller{nullptr};
    SpeedController *m_speed_controller{nullptr};

    /**
     * @brief 控制器程序
     * 
     */
    CurrentController::ControllerLoopFuncType m_controller_loop_func{nullptr};

    float m_bus_voltage_meas; /*!< 母线电压, V */
    float m_bus_current_meas; /*!< 母线电流, A */

    hrt_absnano m_current_sample_ts; /*!< 电流采样时刻，ns */

    float m_effective_current_limit{10.f}; /*!< A */
    float m_max_allowed_current; /*!< A */
    float m_max_dc_calib;
    float m_phase_current_rev_gain; /*!< ADC 输出电压与 Shunt 的比例 */
    float m_power_watt;

    float m_position_est{NAN}; /*!< electrical angle, rad */
    float m_velocity_est{NAN}; /*!< electrical angular velocity, rad/s */

    float m_position_sp{0.f}; /*!< electrical angle, rad */
    float m_velocity_sp{0.f}; /*!< electrical angular velocity, rad/s */
    float m_torque_sp{0.f};   /*!< Nm */
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
