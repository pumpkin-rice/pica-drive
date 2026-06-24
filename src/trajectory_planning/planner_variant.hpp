/**
 * @file planner_factory.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_TRAJECTORY_PLANNER_FACTORY_HPP_
#define _PICA_TRAJECTORY_PLANNER_FACTORY_HPP_

#include "trajectory_planner.hpp"
#include "velocity_trapezoidal.hpp"
#include "velocity_smoothing.hpp"
#include <variant>
#include <stdint.h>

namespace pica
{
    
class TrajectoryPlannerVariant
{
public:
    enum Mode : int
    {
        TRAJ_VEL_TRAP = 0,
        TRAJ_VEL_SMOOTHING = 1,
    };

    TrajectoryPlannerVariant(Mode mode = TRAJ_VEL_TRAP);

    const Trajectory& evaluate(float t)
    {
        m_planner_ptr->evaluate(t);
        // switch (m_planner.index())
        // {
        // case TRAJ_VEL_TRAP:
        //     return std::get<TRAJ_VEL_TRAP>(m_planner).evaluate(t);
        
        // case TRAJ_VEL_SMOOTHING:
        //     return std::get<TRAJ_VEL_SMOOTHING>(m_planner).evaluate(t);
            
        // default:
        //     // TODO: error
        // }
    }

private:
    TrajectoryPlanner *m_planner_ptr{&std::get<TRAJ_VEL_TRAP>(m_planner)};
    std::variant<VelocityTrapezoidal, VelocitySmoothing> m_planner;
};

} // namespace pica


#endif /* !_PICA_TRAJECTORY_PLANNER_FACTORY_HPP_ */
