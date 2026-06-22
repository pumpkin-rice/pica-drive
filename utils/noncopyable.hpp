/**
 * @file noncopyable.hpp
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_NONCOPYABLE_HPP_
#define _PICA_NONCOPYABLE_HPP_

namespace pica
{

class Noncopyable
{
public:
    Noncopyable()  = default;
    ~Noncopyable() = default;
    Noncopyable(const Noncopyable&) = delete;
    Noncopyable& operator=(const Noncopyable&) = delete;
};
    
} // namespace pica


#endif /* !_PICA_NONCOPYABLE_HPP_ */
