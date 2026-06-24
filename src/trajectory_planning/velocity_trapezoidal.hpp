/**
 * @file velocity_trapezoidal.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_VELOCITY_TRAPEZOIDLA_H_
#define _PICA_VELOCITY_TRAPEZOIDLA_H_

#include "trajectory_planner.hpp"

namespace pica
{
    
class VelocityTrapezoidal : public TrajectoryPlanner
{
public:
    bool plan(float pos_sp, float pos_init, float vel_init) override;
    
    const Trajectory& evaluate(float t) override;
};

} // namespace pica

#endif /* !_PICA_VELOCITY_TRAPEZOIDLA_HPP_ */
