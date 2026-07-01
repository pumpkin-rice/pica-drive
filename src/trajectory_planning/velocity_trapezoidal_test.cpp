/**
 * @file test_velocity_trapezoidal.cpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "velocity_trapezoidal.hpp"
#include "planner_variant.hpp"
#include "matplotlibcpp.h"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace pica;

namespace plt = matplotlibcpp;

constexpr float SIGMA = 1e-5;

namespace
{

bool equal(float v1, float v2)
{
    float diff = v1-v2;
    return (std::abs(diff) < SIGMA);
}

bool equal(double v1, double v2)
{
    double diff = v1-v2;
    return (std::abs(diff) < SIGMA);
}

} // namespace name

static void plan(float pos_sp, float pos_init, float vel_init, float accel_max, float vel_max)
{
    std::vector<double> tick, pos, vel, accel, jerk;

    VelocityTrapezoidal planner(vel_max, accel_max);

    planner.plan(pos_sp, pos_init, vel_init);

    for (int i = 0; ; ++i) {
        float t = 1e-1f * i;

        auto& traj = planner.evaluate(t);

        tick.push_back(t);
        pos.push_back(traj.x);
        vel.push_back(traj.v);
        accel.push_back(traj.a);
        jerk.push_back(traj.j);

        if (pos.size() > 2) {
            if (equal(pos[i], pos[i-1])) {
                break;
            }
        }
    }
    plt::figure_size(1200, 780);
    plt::plot(tick, pos);
    plt::plot(tick, vel, "g--");
    plt::plot(tick, accel, "r--");
    plt::show();
}

static void variant(float pos_sp, float pos_init, float vel_init, float accel_max, float vel_max)
{
    std::vector<double> tick, pos, vel, accel, jerk;

    TrajectoryPlannerVariant tpv{TrajectoryPlannerVariant::TRAJ_VEL_TRAP, vel_max, accel_max};
    tpv.plan(pos_sp, pos_init, vel_init);

    for (int i = 0; ; ++i) {
        float t = 1e-1f * i;

        auto& traj = tpv.evaluate(t);

        tick.push_back(t);
        pos.push_back(traj.x);
        vel.push_back(traj.v);
        accel.push_back(traj.a);
        jerk.push_back(traj.j);

        if (pos.size() > 2) {
            if (equal(pos[i], pos[i-1])) {
                break;
            }
        }
    }
    plt::figure_size(1200, 780);
    plt::plot(tick, pos);
    plt::plot(tick, vel, "g--");
    plt::plot(tick, accel, "r--");
    plt::show();
}

TEST(TrajectoryPlanningTest, TrapezoidalTest)
{
    plan(8, 0, 0, 2, 4);
    plan(8, 2, 2, 2, 2);
    plan(8, -8, 2, 2, 4);

    variant(8, 0, 0, 2, 4);
    variant(8, 2, 2, 2, 2);
    variant(8, -8, 2, 2, 4);
}
