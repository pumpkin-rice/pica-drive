/**
 * @file current_controller.h
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_CURRENT_CONTROLLER_H_
#define _PICA_CURRENT_CONTROLLER_H_

#define __STRUCT_CURRENT_CONTROLLER_OBJ \
struct { \
    int (*update_config)(current_controller_obj *motor); \
    int (*update)(current_controller_obj *obj); \
    int (*run)(current_controller_obj *obj, float time2last_sampling, float time2next_output, float period); \
    motor_obj *motor; \
    bool current_loop_enabled; /*!< true: FOC runs in current control mode using Idq_sp, false: FOC runs in voltage control mode using Vdq_sp */ \
}

typedef struct __current_controller_object current_controller_obj;
typedef struct __motor_object motor_obj;

struct __current_controller_object
{
    __STRUCT_CURRENT_CONTROLLER_OBJ;
};

#define CURRENT_CONTROLLER_OBJECT \
struct { \
    __STRUCT_CURRENT_CONTROLLER_OBJ; \
}

#endif /* !_PICA_CURRENT_CONTROLLER_H_ */
