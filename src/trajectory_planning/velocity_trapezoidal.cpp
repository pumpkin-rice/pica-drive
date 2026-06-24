/**
 * @file velocity_trapezoidal.cpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "velocity_trapezoidal.hpp"
#include "math.hpp"

using namespace pica;

bool VelocityTrapezoidal::plan(float pos_sp, float pos_init, float vel_init)
{
    float time_accel, time_vel_const, time_decel;

    float delta_pos = pos_sp - pos_init;

    /**
     * 计算最小减速距离: 无加速和匀速过程时,电机停转所需运动距离
     * 
     * $t = v_0 / a$
     * $s = v_0 t - \frac{1}{2} a * t^2$
     */
    float stop_dist = square(vel_init) / (2.f * (-m_accel_max));
    /* 减速位移 */
    float stop_displacement = std::copysign(stop_dist, vel_init);
    float s = sign_hard(delta_pos - stop_displacement);

    m_accel     = s * m_accel_max;
    m_decel     = -m_accel;
    m_vel_uniform = s * m_vel_max;

    if ((s * vel_init) > (s * m_vel_uniform)) {
        m_accel = -s * m_accel_max;
    }

    // 计算加速,减速时间
    time_accel = (m_vel_uniform - m_vel_init)/m_accel;
    time_decel = -m_vel_uniform/m_decel;

    // 加速距离+减速距离
    float delta_dis_min = 0.5f * time_accel * (m_vel_uniform + vel_init) 
                            + 0.5 * time_decel * m_vel_uniform;

    if (s * delta_pos < s * delta_dis_min) {
        /**
         * 运行距离不足时，按照三角形规划
         */

        m_vel_uniform = s * std::sqrt(
            std::max(
                (time_decel * square(vel_init)
                    + 2 * m_accel * m_decel * delta_pos)/(m_decel - m_accel),
                0.f
            )
        );
        time_accel = std::max(0.f, (m_vel_uniform - vel_init)/m_accel);
        time_decel = std::max(0.f, -m_vel_uniform/m_decel);
        time_vel_const = 0.f;

    } else {
        time_vel_const = (delta_pos - delta_dis_min) / m_vel_uniform;
    }

    m_T1 = time_accel;
    m_T2 = time_accel + time_vel_const;
    m_T3 = m_T2 + time_decel;

    m_pos_init = pos_init;
    m_pos_sp   = pos_sp;
    m_vel_init = vel_init;
    m_pos_accel_final = pos_init
                        + vel_init * time_accel
                        + .5f * m_accel * square(time_accel);

    return true;
}

const Trajectory& VelocityTrapezoidal::evaluate(float t)
{
    m_state.j = 0.f;

    if (t < 0.f) {
        m_state.x = m_pos_init;
        m_state.v = m_vel_init;
        m_state.a = m_state.j = 0.f;

    } else if (t < m_T1) { // 匀加速过程
        m_state.x = m_pos_init + m_vel_init * t + .5f * m_accel * square(t);
        m_state.v = m_vel_init + m_accel * t;
        m_state.a = m_accel;

    } else if (t < m_T2) { // 匀速过程
        m_state.x = m_pos_accel_final + m_vel_uniform * (t-m_T2);
        m_state.v = m_vel_uniform;
        m_state.a = 0.f;

    } else if (t < m_T3) { // 匀减速过程
        float td = t - m_T2;
        m_state.x = m_pos_delta + 0.5f * m_decel * square(td);
        m_state.v = m_decel * td;
        m_state.a = m_decel;

    } else if (t >= m_T3) {
        m_state.x = m_pos_sp;
        m_state.v = m_state.a = 0.f;

    } else {
        // TODO: error
    }

    return m_state;
}
