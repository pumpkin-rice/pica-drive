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

class SpeedControllerVariant
{
public:
    enum ControllerType
    {
        kUnknow = 0,
        kPI = 1, /*!< SpeedControllerPI */
    };

    void *emplace(ControllerType type, BLDC& motor)
    {
#define XX(_label, _type) case (_label) : m_type = (_label); return &m_variant.emplace<_type>(motor);

        switch (type) {
        XX(kPI, SpeedControllerPI);

        case ControllerType::kUnknow:
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
        XX(kPI, SpeedControllerPI);

        case ControllerType::kUnknow:
        default:
            assert(false);
            return false;
        }

#undef XX
    }

    bool update(float *torque_ref, hrt_absnano now)
    {
#define XX(_label, _type) case (_label) : return std::get<_type>(m_variant).update(torque_ref, now)

        switch (m_type) {
        XX(kPI, SpeedControllerPI);

        case ControllerType::kUnknow:
        default:
            return false;
        }

        return false;

#undef XX
    }

private:

private:
    ControllerType m_type;

    std::variant<std::monostate, SpeedControllerPI> m_variant;
};
    
} // namespace bldc


}
    
} // namespace pica


#endif /* !_PICA_DRIVE_BLDC_SPEED_CONTROLLER_HPP_ */
