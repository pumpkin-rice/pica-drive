/**
 * @file motor.c
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "motor/config.hpp"
#include <string.h>
#include <math.h>

void pica::motor::Config::initDefaultValue()
{
    *this = Config{};
}
