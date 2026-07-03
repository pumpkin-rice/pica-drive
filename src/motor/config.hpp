#ifndef _PICA_MOTOR_CONF_MOTOR_PARAM_H_
#define _PICA_MOTOR_CONF_MOTOR_PARAM_H_

#include <stdint.h>
#include <stdbool.h>
#include <cmath>

namespace pica
{

namespace motor
{

struct SpeedControllerConfig
{
    struct {
        float pos_gain = 20.f; /*!< (turn/s)/turn */
        float vel_gain = 1.f/6.f; /*!< Nm/(turn/s) */
        float vel_integrator_gain = 2.0f/6.0f;
    } pi;

    bool vel_limit_enabled = true;
    float vel_limit = INFINITY; /*!< Infinity to disable vel limit, turn/s */
    float vel_limit_tolerance = 1.2f; /*!< ratio to vel_lim. Infinity to disable */
    float vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.

    float gain_scheduling_width = 10.0f;
    bool gain_scheduling_enabled = true;
    bool overspeed_error_enabled = true;
    bool torque_mode_vel_limit_enabled = true;  // enable velocity limit in current control mode (requires a valid velocity estimator)

    float mechanical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for mechanical power for spinout detction
    float electrical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for electrical power for spinout detection
    float spinout_electrical_power_threshold = 10.0f; // [W] electrical power threshold for spinout detection
    float spinout_mechanical_power_threshold = -10.0f; // [W] mechanical power threshold for spinout detection

    int8_t control_mode = -1;
};

/**
 * @brief 
 * 
 * @note for gimbal motors, all units of Nm are instead V.
 * example: vel_gain is [V/(turn/s)] instead of [Nm/(turn/s)]
 * example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
 */
struct Config
{
    SpeedControllerConfig speed_controller_cfg;

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

    uint8_t pole_pairs{5}; /*!< pole piar number, one for DC */
    int8_t motor_type{-1}; /*!< 电机类型, 详细定义见电机子类 @ref{enum Type} */
    int8_t current_controller_type{-1}; /*!< 电流控制器类型, 详细定义见电机子类 @ref{enum CurrentControllerType} */

    void initDefaultValue();
};
    
} // namespace motor

} // namespace pica

#endif /* !_PICA_MOTOR_CONF_MOTOR_PARAM_H_ */
