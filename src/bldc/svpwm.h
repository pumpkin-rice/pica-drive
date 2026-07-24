/**
 * @file svpwm.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_BLDC_SVM_HPP_
#define _PICA_DRIVE_BLDC_SVM_HPP_

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief 计算 SVM 占空比
 * 
 * @param[in] alpha 
 * @param[in] beta 
 * @return int [0..5] SVM 扇区， 错误时返回负值
 */
int svm(float alpha, float beta, float *a, float *b, float *c);

#ifdef __cplusplus
 }
#endif

#endif /* !_PICA_DRIVE_BLDC_SVM_HPP_ */
