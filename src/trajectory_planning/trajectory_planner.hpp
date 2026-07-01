/**
 * @file trajectory_planning.h
 * @author Pumpkin Rice
 * @brief 运动规划生成器（工厂模式）
 * @version 0.1
 * @date 2026-06-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_TRAJECTORY_PLANNING_HPP_
#define _PICA_TRAJECTORY_PLANNING_HPP_

namespace pica
{

struct Trajectory
{
    float j; //< jerk
    float a; //< acceleration
    float v; //< velocity
    float x; //< position
};

/**
 * @brief 按照时间最短原则进行梯形速度规划: 位置曲线为 S 形
 * 
 */
class TrajectoryPlanner
{
public:
    explicit TrajectoryPlanner(float vel_max, float accel_max, float jerk_max)
        : m_jerk_max(jerk_max)
        , m_accel_max(accel_max)
        , m_vel_max(vel_max)
        {}
    ~TrajectoryPlanner() = default;

    virtual bool plan(float pos_sp, float pos_init, float vel_init) = 0;

    /**
     * @brief 更新当前时刻轨迹
     * 
     * @param[in] t 当前轨迹时间： 从调用 plan() 开始计时
     * @return const Trajectory& 
     */
    virtual const Trajectory& evaluate(float t) = 0;

    float getMaxJerk() const  { return m_jerk_max; }
    void setmaxJerk(float j)  { m_jerk_max = j; }
    float getMaxAccel() const { return m_accel_max; }
    void setmaxAccel(float a) { m_accel_max = a; }
    float getMaxVel() const   { return m_vel_max; }
    void setmaxVel(float v)   { m_vel_max = v; }

    float getTotalTime() const { return m_T3; }
    float getUniformVelocity() const { return m_vel_uniform; }

    float getCurrentPosition() const { return m_state.x; }
    float getCurrentVelocity() const { return m_state.v; }
    float getCurrentAcceleration() const { return m_state.a; }
    float getCurrentJerk() const { return m_state.j; }

    /**
     * @brief 根据传入参数,在 t 时间内 估算 急动度,加速度,速度,位置 等数据
     * 
     * @param[in] j jerk
     * @param[in] a0 initial acceleration at t = 0
     * @param v0 initial velocity
     * @param x0 initial postion
     * @param t current time
     * @param d direction
     * @retval Trajectory
     */
    virtual Trajectory evaluatePoly(float j, float a0, float v0, float x0, float t, int d) { return Trajectory{}; };

    void reset(float jerk_max, float accel_max, float vel_max)
    {
        m_jerk_max  = jerk_max;
        m_accel_max = accel_max;
        m_vel_max   = vel_max;

        m_pos_delta = m_pos_sp = 0.f;
        m_vel_uniform = 0.f;
        m_accel = m_decel = 0.f;
        m_pos_accel_final = 0.f;

        m_pos_init = m_vel_init = 0.f;
        
        m_T1 = m_T2 = m_T3 = 0.f;
    }

protected:

    Trajectory m_state{0.f};

    float m_pos_delta{0.f};
    float m_pos_sp{0.f};

    float m_accel{0.f};
    float m_decel{0.f};
    float m_vel_uniform{0.f}; /*!< 匀速段速度 */

    float m_pos_accel_final{0.f}; /*!< 加速段完成时刻位置 */

    float m_pos_init{0.f};
    float m_vel_init{0.f};

    float m_vel_max{0.f};
    float m_jerk_max{0.f};
    float m_accel_max{0.f};

    float m_T1{0.f}; /*!< 加速段完成时刻, s */
    float m_T2{0.f}; /*!< 匀速段完成时刻, s */
    float m_T3{0.f}; /*!< 减速段完成时刻(路径规划完成时刻), s */
};

} // namespace pica

#endif /* !_PICA_TRAPEZOIDLA_TRAJECTORY_H_ */
