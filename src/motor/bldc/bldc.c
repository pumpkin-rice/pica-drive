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
#include "private/math.h"
#include <assert.h>
#include <stddef.h>

extern int foc_init(current_controller_obj *obj, struct bldc *bldc);

static void calc_phase_current_gain(struct bldc *bldc);

int bldc_init(struct bldc *bldc, struct motor_parameters *param)
{
	int ret = 0;

	if (!bldc || !param) {
		return -1;
	}

	bldc->param = param;

	switch (bldc->param->ctrl_type) {
	case MOTOR_CTRL_TYPE_FOC:
		ret = foc_init(&bldc->current_controller, bldc);
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

int bldc_update(struct bldc *bldc)
{
	bldc->current_controller.update(&bldc->current_controller);
}

void bldc_sample_current_calibrator_handler(struct bldc *bldc, const float calib[], float measure_period)
{
	float *c = bldc->dc_current_calibrator;

	if (NULL == calib) {
		c[0] = 0.f;
		c[1] = 0.f;
		c[2] = 0.f;

		bldc->current_calibrator_running_since = 0.f;

	} else {
		const float k_filter = fminf(
			measure_period / bldc->param->dc_current_calibrator_filter_tau, 
			1.f
		);

		c[0] += (calib[0] - c[0]) * k_filter;
		c[1] += (calib[1] - c[1]) * k_filter;
		c[2] += (calib[2] - c[2]) * k_filter;

		bldc->current_calibrator_running_since += measure_period;
	}
}

static float get_effective_current_limit(struct bldc *bldc);

int bldc_do_checks(struct bldc *bldc)
{
	get_effective_current_limit(bldc);
}

float bldc_get_max_available_torque(struct bldc *bldc)
{
	struct motor_parameters *param = bldc->param;
	float torque;

	if (MOTOR_TYPE_ACIM == param->type) {
		// torque = motor->effective_current_limit * param->torque_constant * axis_->acim_estimator_.rotor_flux_;

	} else {
		torque = bldc->effective_current_limit * param->torque_constant;
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
		current_limit = fminf(current_limit, 0.98f * FRAC_1_SQRT3 * (bldc)->bus_voltage_meas); //gimbal motor is voltage control
	
	} else {
		current_limit = fminf(current_limit, 
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
