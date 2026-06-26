/**
 * @file math.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>

namespace pica
{

template<typename T>
constexpr T square(T v)
{
    return v * v;
}

template<typename T>
constexpr T cubic(T v)
{
    return v*v*v;
}

template<typename T>
constexpr T sign_hard(T v)
{
    return std::signbit(v) ? (T)(-1) : (T)(1);
}

} // namespace pica

