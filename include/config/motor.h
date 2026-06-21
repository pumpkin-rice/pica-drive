#ifndef _PICA_MOTOR_CONF_MOTOR_PARAM_H_
#define _PICA_MOTOR_CONF_MOTOR_PARAM_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif

enum motor_type
{
    MOTOR_TYPE_INVALID = 0,
    MOTOR_TYPE_ACIM, /*!< 异步感应 */
    MOTOR_TYPE_GIMBAL, /*!< 云台 */
    MOTOR_TYPE_HIGH_CURRENT, /*!< 大电流 */
};

enum motor_controller_type
{
    MOTOR_CTRL_TYPE_INVALID = 0,
    MOTOR_CTRL_TYPE_FOC,
};

#define MOTOR_PARAM_ACIM_AUTOFLUX_ENABLED         bit(0)
#define MOTOR_PARAM_R_wL_FF_ENABLED               bit(1)
#define MOTOR_PARAM_b_EMF_FF_ENABLED              bit(2)

/**
 * @brief 
 * 
 * @note for gimbal motors, all units of Nm are instead V.
 * example: vel_gain is [V/(turn/s)] instead of [Nm/(turn/s)]
 * example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
 */
struct motor_parameters
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

    enum motor_type type;
    enum motor_controller_type ctrl_type;

    uint8_t pole_pairs; /*!< pole piar number, one for DC */

    bool acim_autoflux_enabled;
    bool R_wL_FF_enabled; /*!< 是否使用 omega 前馈 dq 电流消耦 */
    bool b_EMF_FF_enabled; /*!< 是否使用 omega 前馈补偿 q 轴反电动势 */
};

void motor_init_param_by_default(struct motor_parameters *conf);

#ifdef __cplusplus
 }
#endif

#endif /* !_PICA_MOTOR_CONF_MOTOR_PARAM_H_ */
