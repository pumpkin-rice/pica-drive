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
    float vel_limit = 2.0f; /*!< Infinity to disable vel limit, turn/s */
    float vel_limit_tolerance = 1.2f; /*!< ratio to vel_lim. Infinity to disable */
    float vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.

    uint32_t steps_per_circular_range = 1024;
    float inertia = 0.0f;                    // [Nm/(turn/s^2)]
    float input_filter_bandwidth = 2.0f;     // [1/s]
    float homing_speed = 0.25f;              // [turn/s]
    float gain_scheduling_width = 10.0f;
    bool gain_scheduling_enabled = false;
    bool overspeed_error_enabled = true;
    bool torque_mode_vel_limit_enabled = true;  // enable velocity limit in current control mode (requires a valid velocity estimator)
    uint8_t axis_to_mirror = -1;
    float mirror_ratio = 1.0f;
    float torque_mirror_ratio = 0.0f;
    uint8_t load_encoder_axis = -1;  // default depends on Axis number and is set in load_configuration(). Set to -1 to select sensorless estimator.
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
    
    float torque_constant; // Kt, [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor
    float phase_inductance; /*!< Ls, H */
    float phase_resistance; /*!< Rs, ohm */
    float shunt_conductance; /*!< G, mho */

    float current_limit; /*!< A */
    float current_limit_margin; /*!< Maximum violation of current_lim */
    float current_bus_hard_min;
    float current_bus_hard_max;
    float requested_current_range;

    float current_controller_bandwidth; /*!< rad/s */

    float torque_limit; /*!< Nm */
    
    bool R_wL_FF_enabled; /*!< 是否使用 omega 前馈 dq 电流消耦 */
    bool b_EMF_FF_enabled; /*!< 是否使用 omega 前馈补偿 q 轴反电动势 */

    float acim_gain_min_flux; // [A]
    float acim_autoflux_min_Id; // [A]
    float acim_autoflux_attack_gain;
    float acim_autoflux_decay_gain;

    bool acim_autoflux_enabled;

    float inverter_temp_limit_lower;
    float inverter_temp_limit_upper;

    float current_leak_max;

    float dc_current_calibrator_filter_tau;
    float calibration_current;
    float resistance_calib_max_voltage;

    uint8_t pole_pairs; /*!< pole piar number, one for DC */
    int8_t motor_type; /*!< 电机类型, 详细定义见电机子类 @ref{enum Type} */
    int8_t current_controller_type; /*!< 电流控制器类型, 详细定义见电机子类 @ref{enum CurrentControllerType} */

    void initDefaultValue();
};
    
} // namespace motor

} // namespace pica

#endif /* !_PICA_MOTOR_CONF_MOTOR_PARAM_H_ */
