/**
 * @file hrt_conf.h
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_HRT_PORTS_CONF_H_
#define _PICA_DRIVE_HRT_PORTS_CONF_H_

#include <stdint.h>

/**
 * @brief 电流环运行周期，s
 * 
 */
#define _HRT_CURRENT_MEASURE_PERIOD  (1.f/16e3)

/**
 * @brief 速度环运行周期，s
 * 
 */
#define _HRT_SPEED_MEASURE_PERIOD (1.f/1e3)

#define _HRT_TICK_CONTROL_UPDATE_TO_CURRENT_UPDATE_DELTA_MAX (1)

#endif /* !_PICA_DRIVE_HRT_H_ */
