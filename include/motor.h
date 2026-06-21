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

#include "config/motor.h"
#include "motor/current_controller.h"

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <math.h>

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

struct __motor_object
{
    __STRUCT_MOTOR_INF;
    __STRUCT_MOTOR_DATA data;
};

#define motor_data(_motor) ((_motor)->data)

/**
 * @brief 定义具体电机类型的时候需要在最开始包含此 macro
 * 
 */
#define MOTOR_OBJECT \
struct { \
    __STRUCT_MOTOR_INF; \
    __STRUCT_MOTOR_DATA; \
}

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
int motor_init(motor_obj *motor, struct motor_parameters *param);

/**
 * @brief 更新电流采样数值
 * 
 * @param[in] motor 
 * @param[in] shunt_volt 电流传感器测量得到的实际电压（测流电阻两边电压），多相时按照相序输入
 */
static inline void motor_sample_current_handler(motor_obj *motor,
        const float *shunt_volt)
{
    motor->sample_current_handler(motor, shunt_volt);
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
static inline void motor_sample_current_calibrator_handler(motor_obj *motor,
        const float calibator[], float measure_period)
{
    motor->sample_current_calibrator_handler(motor, calibator, measure_period);
}

static inline void motor_sample_voltage_handler(motor_obj *motor,
        const float *voltage_phases, float voltage_bus)
{
    if (voltage_phases) {
        motor->sample_voltage_handler(motor, voltage_phases);
    }
    
    if (isfinite(voltage_bus)) {
        motor_data(motor).bus_voltage_meas = voltage_bus;
    }
}

/**
 * @brief 更新当前角度数据
 * 
 * @param[in] motor 
 * @param[in] theta_mech rad
 * @param[in] omega_mech rad/s
 */
static inline void motor_sample_encoder_handler(motor_obj *motor,
        float theta_mech, float omega_mech)
{
    uint8_t np = motor->param->pole_pairs;

    motor_data(motor).theta_elec = theta_mech * np;
    motor_data(motor).omega_elec = omega_mech * np;
}

static inline int motor_update(motor_obj *motor)
{
    return motor->update(motor);
}

/**
 * @brief run current control loop
 * 
 * @param[in] motor 
 * @param[in] time2last_sampling time to previous current sampling, second
 * @param[in] time2next_output time to next pwm output, second
 * @return int 
 */
static inline int motor_run_current_loop(motor_obj *motor,
        float time2last_sampling, float time2next_output)
{
    return motor->current_controller->run(
        motor->current_controller,
        time2last_sampling,
        time2next_output
    );
}

static inline void motor_get_duty_cycles(motor_obj *motor, float *duty_cycle)
{
    motor->get_duty_cycles(motor, duty_cycle);
}


static inline int motor_do_checks(motor_obj *motor)
{
    assert(motor && motor->do_checks);
    return motor->do_checks(motor);
}

/**
 * @brief update setpoint of torque
 * 
 * @param[in] motor 
 * @param[in] torque 
 * @return int 
 */
static inline void motor_set_torque(motor_obj *motor, float torque)
{
    motor_data(motor).torque_sp = torque;
}

static inline float motor_max_available_torque(motor_obj *motor)
{
    assert(motor && motor->get_max_available_torque);
    return motor->get_max_available_torque(motor);
}

static inline int motor_update_config(motor_obj *motor)
{
    assert(motor && motor->update_config);

    return motor->update_config(motor);
}

#ifdef __cplusplus
 }
#endif

#endif /* !_PICA_MOTOR_H_ */
