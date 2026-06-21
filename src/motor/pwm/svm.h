/**
 * @file svm.h
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-04-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _MOTOR_SVM_H_
#define _MOTOR_SVM_H_

#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif

bool svm(float alpha, float beta, float *a, float *b, float *c);

#ifdef __cplusplus
 }
#endif

#endif /* !_MOTOR_SVM_H_ */
