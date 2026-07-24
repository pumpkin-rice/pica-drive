/**
 * @file speed_controller.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_BLDC_SPEED_CONTROLLER_VARIANT_HPP_
#define _PICA_DRIVE_BLDC_SPEED_CONTROLLER_VARIANT_HPP_

#include "speed_controller/speed_controller.hpp"
#include "speed_controller/speed_controller_pi.hpp"
#include <variant>
#include <cmath>

namespace pica
{

namespace motor
{

namespace bldc
{

namespace speed
{

enum Type : int8_t
{
    kUnknow = -1,
    kPI = 0, /*!< SpeedControllerPI */
};

} // namespace speed


class SpeedControllerVariant
{
public:
    using Type = speed::Type;

    void *emplace(Type type, BLDC& motor)
    {
#define XX(_label, _type) case (_label) : m_type = (_label); return &m_variant.emplace<_type>(motor);

        switch (type) {
        XX(Type::kPI, SpeedControllerPI);

        case Type::kUnknow:
        default:
            assert(false);
            return nullptr;
        }

#undef XX
    }

    bool init(void *cfg)
    {
#define XX(_label, _type) case (_label) : return std::get<_type>(m_variant).init(cfg)

        switch (m_type) {
        XX(Type::kPI, SpeedControllerPI);

        case Type::kUnknow:
        default:
            assert(false);
            return false;
        }

#undef XX
    }

    bool update(hrt_absnano now, float *torque_ref)
    {
#define XX(_label, _type) case (_label) : return std::get<_type>(m_variant).update(now, torque_ref)

        switch (m_type) {
        XX(Type::kPI, SpeedControllerPI);

        case Type::kUnknow:
        default:
            return false;
        }

        return false;

#undef XX
    }

private:

private:
    Type m_type;

    std::variant<std::monostate, SpeedControllerPI> m_variant;
};
    
} // namespace bldc


}
    
} // namespace pica


#endif /* !_PICA_DRIVE_BLDC_SPEED_CONTROLLER_HPP_ */
