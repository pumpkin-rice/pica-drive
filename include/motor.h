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
    struct motor_parameters *param; \
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

/**
 * @brief 定义具体电机类型的时候需要在最开始包含此 macro
 * 
 */
#define MOTOR_OBJECT \
struct { \
    /* __STRUCT_MOTOR_INF; */ \
    __STRUCT_MOTOR_DATA; \
}

struct __motor_object
{
    MOTOR_OBJECT;
};

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif

#endif /* !_PICA_MOTOR_H_ */
