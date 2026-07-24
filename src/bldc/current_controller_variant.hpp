/**
 * @file current_controller_variant.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_BLDC_CURRENT_CONTROLLER_VARIANT_HPP_
#define _PICA_DRIVE_BLDC_CURRENT_CONTROLLER_VARIANT_HPP_

#include "current_controller/current_controller_foc.hpp"
#include "current_controller/phase_inductance_measurer.hpp"
#include "current_controller/phase_resistance_measurer.hpp"

#include <variant>

namespace pica
{

namespace motor
{

namespace bldc
{

namespace current
{

enum Type : int8_t
{
    kUnknow = 0,
    kFOC,

    // 仅用于初始化检测阶段，无需多态实现
    // kPhaseInductanceMeasurer,
    // kPhaseResistanceMeasurer,

    _kTypeMax,
};
    
} // namespace current


class CurrentControllerVariant
{
public:
    using Type = current::Type;

    void *emplace(Type type, BLDC& bldc)
    {
        switch (type)
        {
#define XX(_l, _T) case (_l): m_type = type; return &m_variant.emplace<_T>(bldc)
        
        XX(Type::kFOC, FOC);

#undef XX

        case Type::kUnknow:
        default:
            return nullptr;
        }
    }

private:
    Type m_type{Type::kUnknow};
    std::variant<std::monostate, FOC, PhaseInductanceMeasurer, PhaseResistanceMeasurer> m_variant;
};
    
} // namespace bldc

    
} // namespace motor

} // namespace pica


#endif /* !_PICA_DRIVE_BLDC_CURRENT_CONTROLLER_VARIANT_HPP_ */
