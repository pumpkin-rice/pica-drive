/**
 * @file bldc.c
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bldc.h"
#include "utils/math.h"
#include <assert.h>
#include <stddef.h>

typedef struct __bldc_private bldc_private;

static int  bldc_update(motor_obj *motor);

static void bldc_sample_current_handler(motor_obj *motor, const float *shunt_volt);

static void bldc_sample_current_calibrator_handler(motor_obj *motor, const float calib[], float measure_period);

static void bldc_sample_voltage_handler(motor_obj *motor, const float *voltage_phases);

static int bldc_update_config(motor_obj *motor);

static int bldc_do_checks(motor_obj *motor);

static inline void bldc_get_duty_cycles(motor_obj *motor, float *duty_cycle)
{
	float *data = ((struct bldc *)motor)->__private.duty_cycle;

	duty_cycle[0] = data[0];
	duty_cycle[1] = data[1];
	duty_cycle[2] = data[2];
}

static float bldc_get_max_available_torque(motor_obj *motor);

extern int foc_init(current_controller_obj *obj, motor_obj *motor);

static void calc_phase_current_gain(struct bldc *bldc);

int bldc_init(motor_obj *motor)
{
	struct bldc *bldc = motor2bldc(motor);
	int ret = 0;

	bldc->update = bldc_update;
	bldc->sample_current_handler = bldc_sample_current_handler;
	bldc->sample_current_calibrator_handler =
			bldc_sample_current_calibrator_handler;
	bldc->sample_voltage_handler = bldc_sample_voltage_handler;
	bldc->update_config = bldc_update_config;
	bldc->get_duty_cycles = bldc_get_duty_cycles;
	bldc->get_max_available_torque = bldc_get_max_available_torque;
	bldc->do_checks = bldc_do_checks;

	switch (bldc->param->ctrl_type) {
	case MOTOR_CTRL_TYPE_FOC:
		ret = foc_init(&bldc_current_obj(bldc), motor);
		break;

	case MOTOR_CTRL_TYPE_INVALID:
	default:
		assert(0);
	}
	// init failed
	if (ret < 0) {
		goto exit;
	}

	bldc->effective_current_limit = 10;

	calc_phase_current_gain(bldc);

exit:
	return ret;
}

int bldc_update(motor_obj *motor)
{
	struct bldc *bldc = motor2bldc(motor);
	current_controller_obj *current_controller = &bldc_current_obj(bldc);

	current_controller->update(current_controller);
}

int bldc_run(motor_obj *motor)
{
	struct bldc *bldc = motor2bldc(motor);
	current_controller_obj *current_controller = &bldc_current_obj(bldc);

	current_controller->update(current_controller);
}

void bldc_sample_current_handler(motor_obj *motor, const float *shunt_volt)
{
	float *c = motor2bldc(motor)->__private.current_meas;

	// float shunt_gain = motor->param->shunt_conductance * motor->phase_current_rev_gain;

	float shunt_gain = 1.f;

	// TODO: calibrate current

	// 实际电压： adc 测得电压* motor->phase_current_rev_gain
	// 实际电流： 实际电压 / 电阻 
	c[0] = shunt_volt[0] * shunt_gain;
	c[1] = shunt_volt[1] * shunt_gain;
	c[2] = shunt_volt[2] * shunt_gain;
}

void bldc_sample_current_calibrator_handler(motor_obj *motor, const float calib[], float measure_period)
{
	bldc_private *motor_data = &bldc_private(motor2bldc(motor));
	float *c = motor_data->dc_current_calibrator;

	if (NULL == calib) {
		c[0] = 0.f;
		c[1] = 0.f;
		c[2] = 0.f;

		motor_data->current_calibrator_running_since = 0.f;

	} else {
		const float k_filter = min(
			measure_period / motor->param->dc_current_calibrator_filter_tau, 
			1.f
		);

		c[0] += (calib[0] - c[0]) * k_filter;
		c[1] += (calib[1] - c[1]) * k_filter;
		c[2] += (calib[2] - c[2]) * k_filter;

		motor_data->current_calibrator_running_since += measure_period;
	}
}

void bldc_sample_voltage_handler(motor_obj *motor, const float *voltage_phases)
{
	if (voltage_phases) {
		float *v = motor2bldc(motor)->__private.voltage_meas;

		v[0] = voltage_phases[0];
		v[1] = voltage_phases[1];
		v[2] = voltage_phases[2];
	}
}

void bldc_sample_encoder_handler(motor_obj *motor, float theta_mach, float omega_mach)
{

}

int bldc_update_config(motor_obj *motor)
{
	motor->current_controller->update_config(motor->current_controller);
}

static float get_effective_current_limit(struct bldc *bldc);

int bldc_do_checks(motor_obj *motor)
{
	struct bldc *bldc = motor2bldc(motor);

	get_effective_current_limit(bldc);
}

float bldc_get_max_available_torque(motor_obj *motor)
{
	struct motor_parameters *param = motor->param;
	float torque;

	if (MOTOR_TYPE_ACIM == param->type) {
		// torque = motor->effective_current_limit * param->torque_constant * axis_->acim_estimator_.rotor_flux_;

	} else {
		torque = motor_data(motor).effective_current_limit * param->torque_constant;
	}

	torque = clamp(torque, 0.f, param->torque_limit);

	return torque;
}

float get_effective_current_limit(struct bldc *bldc)
{
	struct motor_parameters *param = bldc->param;

	float current_limit = param->current_limit; // Configured limit

	// Hardware limit
	if (MOTOR_TYPE_GIMBAL == param->type) {
		current_limit = min(current_limit, 0.98f * FRAC_1_SQRT3 * (bldc)->bus_voltage_meas); //gimbal motor is voltage control
	
	} else {
		current_limit = min(current_limit, 
							bldc->max_allowed_current);
	}

	// TODO: thermistor
	// Apply thermistor current limiters
    // current_lim = std::min(current_lim, motor_thermistor_.get_current_limit(config_.current_lim));
    // current_lim = std::min(current_lim, fet_thermistor_.get_current_limit(config_.current_lim));

	bldc->effective_current_limit = current_limit;

	return current_limit;
}

void calc_phase_current_gain(struct bldc *bldc)
{
	// TODO: 电流限制传感器
	// fet_thermistor_.update();
	// motor_thermistor_.update();

	// Solve for exact gain, then snap down to have equal or larger range as requested
	// or largest possible range otherwise
	const float k_margin = 0.9f;
	const float max_output_swing = 1.35f; /*!< out of amplifier, V */

	// 可测量最大电流 = 限幅 * ADC输出范围 * 电导
	float max_unity_gain_current =
			k_margin * max_output_swing * bldc->param->shunt_conductance;
	// 需用增益 = 最大可测电流 / 最大电流
	float requested_gain = max_unity_gain_current / bldc->param->requested_current_range;

	float actual_gain; /*!< 实际增益 */
	if (0) {
	// if (!gate_driver_.config(requested_gain, &actual_gain))
	// 		return false;
	} else {
		actual_gain = requested_gain;
	}

	// 实际电压： adc 测得电压* motor->phase_current_rev_gain
	// 实际电流： 实际电压 / 电阻 
	bldc->phase_current_rev_gain = 1.f/actual_gain;
	bldc->max_allowed_current = max_unity_gain_current
			* bldc->phase_current_rev_gain;

	bldc->max_dc_calib = 0.1f * bldc->max_allowed_current;
}
