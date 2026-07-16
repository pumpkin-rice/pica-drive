/**
 * @file drive_conf.h
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_CONF_H_
#define _PICA_DRIVE_CONF_H_

/**
 * @brief PWM 生成方式：SVPWM
 * 
 */
#define PICA_DRIVE_PWM_GENERNATOR_SVPWM      0
/**
 * @brief PWM 生成方式：IClakre
 * 
 */
#if GENERNATE_PWM_BY_SVPWM == 1
    #define PICA_DRIVE_PWM_GENERNATOR_ICLARKE    0
#else
    #define PICA_DRIVE_PWM_GENERNATOR_ICLARKE    1
#endif

/**
 * @brief 电流环采样周期，s
 * 
 */
#define PICA_DRIVE_CURRENT_MEASURE_PERIOD       (1.f/16e3f)
/**
 * @brief 速度环运行周期，s
 * 
 */
#define PICA_DRIVE_SPEED_LOOP_PERIOD \
        (PICA_DRIVE_CURRENT_MEASURE_PERIOD)

/**
 * @brief 控制回路运行时刻(update)至电流采样的最大时间间隔，ns
 * 
 */
#define PICA_CONTROLLER_LOOP_UPDATE_TO_CURRENT_MEAS_DELTA_MAX_NANO (120)

#endif /* !_PICA_DRIVE_CONF_H_ */
