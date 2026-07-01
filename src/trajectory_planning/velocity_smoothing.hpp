/**
 * @file velocity_smoothing.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_TRAJECTORY_VELOCITY_SMOOTHING_H_
#define _PICA_TRAJECTORY_VELOCITY_SMOOTHING_H_

#include "trajectory_planner.hpp"

namespace pica
{

class VelocitySmoothing : public TrajectoryPlanner
{
public:
    bool plan(float pos_sp, float pos_init, float vel_init) override;
    const Trajectory& evaluate(float t) override;

private:

};

} // namespace pica


#endif /* !_PICA_TRAJECTORY_VELOCITY_SMOOTHING_H_ */
