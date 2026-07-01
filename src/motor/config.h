#ifndef _PICA_MOTOR_CONF_MOTOR_PARAM_H_
#define _PICA_MOTOR_CONF_MOTOR_PARAM_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief 
 * 
 * @note for gimbal motors, all units of Nm are instead V.
 * example: vel_gain is [V/(turn/s)] instead of [Nm/(turn/s)]
 * example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
 */
struct motor_config
{
    float torque_constant; // Kt, [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor
    float phase_inductance; /*!< Ls, H */
    float phase_resistance; /*!< Rs, ohm */

    float current_limit; /*!< A */
    float current_limit_margin; /*!< Maximum violation of current_lim */
    float current_bus_hard_min;	
    float current_bus_hard_max;
    float current_leak_max;

    float torque_limit; /*!< Nm */
    float requested_current_range;
    float shunt_conductance; /*!< G, mho */

    float dc_current_calibrator_filter_tau;
    float calibration_current;
    float resistance_calib_max_voltage;

    float current_controller_bandwidth; /*!< rad/s */

    float inverter_temp_limit_lower;
    float inverter_temp_limit_upper;

    float acim_gain_min_flux; // [A]
    float acim_autoflux_min_Id; // [A]
    float acim_autoflux_attack_gain;
    float acim_autoflux_decay_gain;

    uint8_t motor_type; /*!< 电机类型, 详细定义见电机子类 @ref{enum Type} */
    uint8_t current_controller_type; /*!< 电流控制器类型, 详细定义见电机子类 @ref{enum CurrentControllerType} */

    uint8_t pole_pairs; /*!< pole piar number, one for DC */

    bool acim_autoflux_enabled;
    bool R_wL_FF_enabled; /*!< 是否使用 omega 前馈 dq 电流消耦 */
    bool b_EMF_FF_enabled; /*!< 是否使用 omega 前馈补偿 q 轴反电动势 */
};

void motor_init_param_by_default(struct motor_config *conf);

#ifdef __cplusplus
 }
#endif

#endif /* !_PICA_MOTOR_CONF_MOTOR_PARAM_H_ */
