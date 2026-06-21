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
#include "motor/bldc/foc.h"

struct __bldc_private
{
    union {
        current_controller_obj obj;
        struct foc foc; /*!< FOC 控制器 */
    } __current_controller_instances; /*!< 电流控制器 */

    float voltage_meas[3]; /*!< measured voltage for [a, b, c], V */
    float current_meas[3]; /*!< measured current for [a, b, c], A */
    float dc_current_calibrator[3]; /*!< A */

    float current_calibrator_running_since; // current sensor calibration needs some time to settle

    float duty_cycle[3]; /*!< PWM duty cycles for A,B,C. value \in [0~1]*/
    
    uint32_t current_sampling_ts; /*!< timer tick */
};

struct bldc
{
    MOTOR_OBJECT;
    struct __bldc_private __private;
};

#define motor2bldc(_motor)  ((struct bldc *)(_motor))

#define bldc_private(_bldc) ((_bldc)->__private)
#define bldc_current_obj(_bldc) (bldc_private(_bldc).__current_controller_instances.obj)

#ifdef __cplusplus
 extern "C" {
#endif

inline float *bldc_get_duty_cycle(struct bldc *bldc)
{
    return bldc_private(bldc).duty_cycle;
}

/*********************** 调试接口 ********************************/
#if ENABLE_PICA_DRIVE_DEBUG == 1

#define bldc_get_foc(_bldc) (bldc_private(_bldc).__current_controller_instances.foc)

inline float *bldc_dbg_voltage_measured(struct bldc *bldc)
{
    return bldc_private(bldc).voltage_meas;
}

inline float *bldc_dbg_current_measured(struct bldc *bldc)
{
    return bldc_private(bldc).current_meas;
}

inline float *bldc_dbg_current_calibrator(struct bldc *bldc)
{
    return bldc_private(bldc).dc_current_calibrator;
}

#endif /*! ENABLE_PICA_DRIVE_DEBUG == 1 */


#ifdef __cplusplus
 }
#endif

#endif /* !_PICA_MOTOR_BLDC_H_ */
