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
// #include "velocity_smoothing.hpp"
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

    TrajectoryPlannerVariant() = delete;
    TrajectoryPlannerVariant(Mode mode = TRAJ_VEL_TRAP, float vel_max = 0.f, float accel_max = 0.f, float jerk_max = 0.f)
    {
        reset(mode, vel_max, accel_max, jerk_max);
    }

    bool plan(float pos_sp, float pos_init, float vel_init)
    {
        return m_planner_ptr->plan(pos_sp, pos_init, vel_init);
    }

    const Trajectory& evaluate(float t)
    {
        return m_planner_ptr->evaluate(t);
    }

    void reset(Mode mode, float vel_max, float accel_max, float jerk_max = 0.f)
    {
        switch (mode) {
        case TRAJ_VEL_TRAP:
            m_planner = VelocityTrapezoidal(vel_max, accel_max);
            m_planner_ptr = &std::get<TRAJ_VEL_TRAP>(m_planner);
            break;
        
        case TRAJ_VEL_SMOOTHING:
        default:
            return ;
        }
    }

private:
    TrajectoryPlanner *m_planner_ptr{&std::get<TRAJ_VEL_TRAP>(m_planner)};
    std::variant<VelocityTrapezoidal> m_planner{VelocityTrapezoidal(0, 0)};
};

} // namespace pica


#endif /* !_PICA_TRAJECTORY_PLANNER_FACTORY_HPP_ */
