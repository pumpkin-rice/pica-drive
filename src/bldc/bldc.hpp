/**
 * @file bldc.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_BLDC_HPP_
#define _PICA_DRIVE_BLDC_HPP_

#include "bldc/park.h"
#include "bldc/svpwm.h"
#include "bldc/current_controller_variant.hpp"
#include "bldc/speed_controller_variant.hpp"
#include "interface/motor.hpp"
#include <cmath>

namespace pica::motor::bldc
{

inline constexpr int16_t genBLDCType(int16_t sub_val)
{
    if (sub_val < 0 || sub_val >=256)
    {
        return -Motor::kBasicTypeBLDC;
    }

    return ((Motor::kBasicTypeBLDC << 8) | sub_val);
}

class PhaseResistanceMeasurer;

class BLDC : public Motor
{
    friend class FOC;
    friend class PhaseResistanceMeasurer;
    friend class PhaseInductanceMeasurer;

    friend class SpeedControllerPI;
    friend class SpeedControllerVariant;

public:
    using CurrentControllerType = CurrentControllerVariant::Type;

    enum Type : int16_t
    {
        kACIM           = genBLDCType(0), /*!< 异步感应 */
        kGimbal         = genBLDCType(1), /*!< 云台 */
        kHighCurrent    = genBLDCType(2), /*!< 大电流 */
    };

    struct Config
    {
        float inertia{0.0000177245f};
        float torque_constant{0.04f}; // Kt, [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor
        float phase_inductance{0.64/2 * 1e-3}; /*!< Ls, H */
        float phase_resistance{0.57/2}; /*!< Rs, ohm */
        float shunt_conductance{1/phase_resistance}; /*!< G, mho */

        float current_limit{10.f}; /*!< A */
        float current_limit_margin{8.f}; /*!< Maximum violation of current_lim */
        float current_bus_hard_min{-INFINITY};
        float current_bus_hard_max{INFINITY};
        float requested_current_range{10.f};

        float current_controller_bandwidth{NAN}; /*!< rad/s */

        float torque_limit{INFINITY}; /*!< Nm */
        
        bool R_wL_FF_enabled{true}; /*!< 是否使用 omega 前馈 dq 电流消耦 */
        bool b_EMF_FF_enabled{true}; /*!< 是否使用 omega 前馈补偿 q 轴反电动势 */

        float acim_gain_min_flux{NAN}; // [A]
        float acim_autoflux_min_Id{NAN}; // [A]
        float acim_autoflux_attack_gain{NAN};
        float acim_autoflux_decay_gain{NAN};

        bool acim_autoflux_enabled{false};

        float inverter_temp_limit_lower{NAN};
        float inverter_temp_limit_upper{NAN};

        float current_leak_max{NAN};

        float dc_current_calibrator_filter_tau{0.2f};
        float calibration_current{10.f};
        float resistance_calib_max_voltage{2.f};

        bool vel_limit_enabled = true;
        float vel_limit = INFINITY; /*!< Infinity to disable vel limit, turn/s */
        float vel_limit_tolerance = 1.2f; /*!< ratio to vel_lim. Infinity to disable */
        bool overspeed_error_enabled = true;
        bool torque_mode_vel_limit_enabled = true;  // enable velocity limit in current control mode (requires a valid velocity estimator)

        float mechanical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for mechanical power for spinout detction
        float electrical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for electrical power for spinout detection
        float spinout_electrical_power_threshold = 10.0f; // [W] electrical power threshold for spinout detection
        float spinout_mechanical_power_threshold = -10.0f; // [W] mechanical power threshold for spinout detection

        int16_t motor_type{-1}; /*!< 电机类型, 详细定义见电机子类 @ref{enum Type} */
        uint8_t pole_pairs{1}; /*!< pole piar number, one for DC */
        int8_t current_controller_type{-1}; /*!< 电流控制器类型, 详细定义见电机子类 @ref{enum CurrentControllerType} */
        int8_t speed_controller_type{-1};
        int8_t motor_control_mode{-1}; /*!< 电机控制模式：力矩、速度、位置 */
    };

    bool init() final;

    bool update(hrt_absnano ts) final;

    bool run(hrt_absnano ts_output) final
    {
        return m_current_controller_proxy.run(ts_output, &m_v_alpha_beta_final);
    }

    void sampleCurrentHandler(const float shunt_volt[], hrt_absnano ts) final
    {
        float shunt_gain = m_cfg.shunt_conductance * m_phase_current_rev_gain;

        m_current_meas = {
            shunt_gain * shunt_volt[0],
            shunt_gain * shunt_volt[1],
            shunt_gain * shunt_volt[2]
        };

        m_ts_current_meas = ts;
    }

    const ABC& currentMeasured() const { return m_current_meas; }

    /**
     * @brief 校正电流采样更新
     * 
     * @param[in] shunt_volt 
     */
    void sampleCurrentCalibratorHandler(const float shunt_volt[], float period) final
    {
        if (nullptr == shunt_volt) {
            m_current_calibrator = {0.f, 0.f, 0.f};
            m_current_calibrator_running_since = 0.f;

        } else {
            ABC calib{shunt_volt};
            const float k_filter = std::min(period/m_cfg.dc_current_calibrator_filter_tau,
                1.f
            );

            m_current_calibrator += (calib - m_current_calibrator) * k_filter;

            m_current_calibrator_running_since += period;
        }
    }

    const AlphaBeta& voltageAlphaBetaFinal() const
    {
        return m_v_alpha_beta_final;
    }


    template<class T>
    T *currentController()
    {
        return m_current_controller_proxy.getInstance<T>();
    }

    const float *dutyCycle() final { return m_duty_cycle; }

    int8_t getSVMSector() const { return m_svm_sector; }

    float getMaxAvailableTorque() final;
    float getEffectiveCurrentLimit();

    int16_t type() const
    {
        return m_cfg.motor_type;
    }

    ControlMode getControlMode() const final
    {
        return (ControlMode)m_cfg.motor_control_mode;
    }

    bool do_checks() { return true; }

    bool arm(void *current_controller) { return true; }
    bool disarm() final;

#if (PICA_DRIVE_ENABLE_DEBUG == 1)

    float getPhaseCurrentRevGain() const { return m_phase_current_rev_gain; }

    template<class T>
    T *getCurrentController()
    {
        return m_current_controller_proxy.getInstance<T>();
    }
    
#endif

private:
    void calcPhaseCurrentGain();

    bool calcSVM(const AlphaBeta& ab)
    {
        m_svm_sector = ::svm(
            ab(0), ab(1), 
            &m_duty_cycle[0],
            &m_duty_cycle[1],
            &m_duty_cycle[2]
        );

        return m_svm_sector >= 0;
    }

    bool measurePhaseResistance(float test_current, float max_voltage);


private:
    Config m_cfg;

    ABC m_current_meas; /*!< 电流采样, A */
    ABC m_current_calibrator; /*!< 校正电流，A */

    float m_duty_cycle[3]; /*!< PWM 占空比，0 for 0, 1 for 100% */

    AlphaBeta m_v_alpha_beta_final{0.f, 0.f}; /*!< 由电流环生成的 alpha-beta 电压指令，用于 PWM 生成和角度观测 */

    float m_current_calibrator_running_since; /*!< current sensor calibration needs some time to settle */
    float m_phase_current_rev_gain;
    float m_max_allowed_current;
    float m_max_dc_calib;

    int8_t m_svm_sector{-1}; /*!< 当前所在 SVM 扇区，正常范围为 0-5,错误时输出为 -1 */

    SpeedControllerVariant m_speed_controller;
    
    FOC m_current_controller{*this}; /*!< 为电流控制器预先分配内存 */
    CurrentControllerProxy m_current_controller_proxy{&m_current_controller}; /*!< 电流控制器句柄 */
};
    
} // namespace pica::motor

#endif 
