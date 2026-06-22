/**
 * @file bldc.h
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_MOTOR_BLDC_H_
#define _PICA_MOTOR_BLDC_H_

#include "motor.h"
#include "private/math.h"
#include "motor/bldc/foc.h"

struct bldc
{
    MOTOR_OBJECT;

    union {
        current_controller_obj current_controller;
        struct foc foc; /*!< FOC 控制器 */
    }; /*!< 电流控制器 */

    float voltage_meas[3]; /*!< measured voltage for [a, b, c], V */
    float current_phases_meas[3]; /*!< measured current for [a, b, c], A */
    float dc_current_calibrator[3]; /*!< A */

    float current_calibrator_running_since; // current sensor calibration needs some time to settle

    float duty_cycle[3]; /*!< PWM duty cycles for A,B,C. value \in [0~1]*/
    
    uint32_t current_sampling_ts; /*!< timer tick */
};

#define motor2bldc(_motor)  ((struct bldc *)(_motor))

#define bldc_current_obj(_bldc) ((_bldc)->current_controller)

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief 电机参数初始化
 * 
 * @param[in] motor 
 * @param[in] param 
 * @return int 
 */
int bldc_init(struct bldc *bldc, struct motor_parameters *param);

/**
 * @brief 更新电流采样数值
 * 
 * @param[in] motor 
 * @param[in] shunt_volt 电流传感器测量得到的实际电压（测流电阻两边电压），多相时按照相序输入
 */
static inline void bldc_sample_current_handler(struct bldc *bldc,
    const float *shunt_volt)
{
    float *c = bldc->current_phases_meas;
    float shunt_gain = bldc->param->shunt_conductance * bldc->phase_current_rev_gain;

    c[0] = shunt_volt[0] * shunt_gain;
    c[1] = shunt_volt[0] * shunt_gain;
    c[2] = shunt_volt[0] * shunt_gain;
}

/**
 * @brief 更新校正电流
 * 
 * @details 更新完成后会自动计算采样电流数值
 * 
 * @param[in] motor 
 * @param[in] calibator 
 * @param[in] measure_period 校正电流测量周期, s
 */
void bldc_sample_current_calibrator_handler(struct bldc *bldc,
        const float calibator[], float measure_period);

static inline void bldc_sample_voltage_handler(struct bldc *bldc,
        const float *voltage_phases, float voltage_bus)
{
    if (voltage_phases) {
		float *v = bldc->voltage_meas;

		v[0] = voltage_phases[0];
		v[1] = voltage_phases[1];
		v[2] = voltage_phases[2];
	}
    
    if (isfinite(voltage_bus)) {
        bldc->bus_voltage_meas = voltage_bus;
    }
}

/**
 * @brief 更新当前角度数据
 * 
 * @param[in] motor 
 * @param[in] theta_mech 
 * @param[in] omega_mech 
 */
static inline void bldc_sample_encoder_handler(struct bldc *bldc,
        float theta_mech, float omega_mech)
{
    uint8_t np = bldc->param->pole_pairs;

    bldc->theta_elec = wrap_pm(theta_mech, M_PI);
    bldc->omega_elec = wrap_pm(omega_mech, M_PI);
}

int bldc_update(struct bldc *bldc);

/**
 * @brief 运行电流环控制程序
 * 
 * @param[in] motor 
 * @param[in] time2last_sampling time to previous current sampling, second
 * @param[in] time2next_output time to next pwm output, second
 * @return int 
 */
static inline int bldc_run(struct bldc *bldc,
        float time2last_sampling, float time2next_output)
{
    return bldc->current_controller.run(
        &bldc->current_controller,
        time2last_sampling,
        time2next_output
    );
}

static inline const float *bldc_get_duty_cycle(struct bldc *bldc)
{
    return bldc->duty_cycle;
}

int bldc_do_checks(struct bldc *bldc);

/**
 * @brief update setpoint of torque
 * 
 * @param[in] motor 
 * @param[in] torque 
 * @return int 
 */
static inline void bldc_set_torque(struct bldc *bldc, float torque)
{
    bldc->torque_sp = torque;
}

float bldc_get_max_available_torque(struct bldc *bldc);

/**
 * @brief 更新电流控制器参数
 * 
 * @param[in] bldc 
 * @return int 
 */
static inline int bldc_update_config(struct bldc *bldc)
{
    return bldc->current_controller.update_config(&bldc->current_controller);
}

static inline struct foc *bldc_get_current_controller_foc(struct bldc *bldc)
{
    return &bldc->foc;
}

static inline const float *bldc_get_current_phase_meas(struct bldc *bldc)
{
    return bldc->current_phases_meas;
}

#ifdef __cplusplus
 }
#endif

#endif /* !_PICA_MOTOR_BLDC_H_ */
