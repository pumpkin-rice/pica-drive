/**
 * @file motor.c
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "motor.h"
#include <string.h>
#include <math.h>

void motor_init_param_by_default(struct motor_parameters *conf)
{
    memset(conf, 0, sizeof(*conf));
    
    conf->type = MOTOR_TYPE_INVALID;

    conf->pole_pairs = 7;
    conf->phase_resistance = 0.f;
    conf->phase_inductance = 0.f;

    conf->torque_constant = 0.04f;
    
    conf->calibration_current = 10.f;
    conf->dc_current_calibrator_filter_tau = 0.2f;
    conf->resistance_calib_max_voltage = 2.f;

    conf->current_limit = 10.f;
    conf->current_limit_margin = 8.f;
    conf->torque_limit = INFINITY;

    // Value used to compute shunt amplifier gains
    conf->requested_current_range = 10.f;
    conf->current_controller_bandwidth = 1e3f;
    conf->inverter_temp_limit_lower = 100;
    conf->inverter_temp_limit_upper = 120;

    conf->acim_autoflux_enabled = false;
    conf->acim_gain_min_flux = 10.f;
    conf->acim_autoflux_min_Id = 10.f;
    conf->acim_autoflux_attack_gain = 10;
    conf->acim_autoflux_decay_gain = 1.f;

    conf->R_wL_FF_enabled  = true;
    conf->b_EMF_FF_enabled = true;

    conf->current_bus_hard_max = INFINITY;
    conf->current_bus_hard_min = -INFINITY;
    conf->current_leak_max = 0.1f;
}
