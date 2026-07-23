/**
 * @file current_controller.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_MOTOR_CURRENT_CONTROLLER_HPP_
#define _PICA_DRIVE_MOTOR_CURRENT_CONTROLLER_HPP_

#include "hrt.h"
#include "bldc/park.h"
#include "utils/noncopyable.hpp"

namespace pica
{

namespace motor
{

namespace bldc
{

class BLDC;

template<class Derived>
class CurrentController : Noncopyable
{
    friend class CurrentControllerProxy;
public:
    CurrentController(BLDC& motor) : m_motor(motor) {}

private:
    static void Reset(void *ctx)
    {
        reinterpret_cast<Derived*>(ctx)->reset();
    }

    static bool Init(void *ctx, void *cfg)
    {
        return reinterpret_cast<Derived*>(ctx)->init(cfg);
    }

    static bool Run(void *ctx, hrt_absnano ts_output,
            AlphaBeta *v_alpha_beta_final)
    {
        return reinterpret_cast<Derived*>(ctx)->run(ts_output, v_alpha_beta_final);
    }

    static bool Update(void *ctx, hrt_absnano now)
    {
        return reinterpret_cast<Derived*>(ctx)->update(now);
    }

    hrt_absnano timestampUpdate() const { return m_ts_update; }

protected:
    BLDC& m_motor;

    bool m_current_loop_enabled{false}; /*!< 是否使用电流环: true - 使用电流环生成控制电压; false - 使用电压环*/

    hrt_absnano m_ts_update{0}; /*!< 环路更新时间(update)，tick */
};

/**
 * @brief 电流控制器代理接口，通过该接口实现多态（类似C的多态方案），相比于 Variant 实现更加简单
 * 
 * @note 这种方式必然伴随代码膨胀
 * 
 */
class CurrentControllerProxy : Noncopyable
{
public:
    typedef bool (*InitFuncType)(void *ctx, void *cfg);
    typedef void (*ResetFuncType)(void *ctx);
    typedef bool (*UpdateFuncType)(void *ctx, hrt_absnano now);
    typedef bool (*RunFuncType)(void *ctx, hrt_absnano ts_output, AlphaBeta *v_alpha_beta_final);

    CurrentControllerProxy() = default;

    template<class T>
    CurrentControllerProxy(T *current_controller)
    {
        reset<T>(current_controller);
    }

    /**
     * @brief 重设代理目标
     * 
     * @tparam T 
     * @param[in] current_controller 
     */
    template<class T>
    void reset(T *current_controller)
    {
        assert(current_controller != nullptr);

        m_ctx = &current_controller;

        m_reset = T::Reset;
        m_init = T::Init;
        m_run = T::Run;
        m_update = T::Update;
    }

    /**
     * @brief 初始化控制器
     * 
     * @param[in] cfg 电流控制器配置参数
     * @return true 
     * @return false 
     */
    bool init(void *cfg) { return m_init(m_ctx, cfg); }

    /**
     * @brief 重置控制器参数
     * 
     */
    void reset() { m_reset(m_ctx); }

    /**
     * @brief 刷新控制器状态
     * 
     * @param[in] now 
     * @return true 
     * @return false 
     */
    bool update(hrt_absnano now) { return m_update(m_ctx, now); }

    /**
     * @brief 运行电流环
     * 
     * @param[in] ts_pwm_output 下一次PWM输出时间
     * @param[in] v_alpha_beta_final 电流控制器输出，alpha-beta 坐标系下电压，V
     * @return true 
     * @return false 
     */
    bool run(hrt_absnano ts_pwm_output, AlphaBeta *v_alpha_beta_final)
    {
        return m_run(m_ctx, 
                        ts_pwm_output,
                        v_alpha_beta_final
                    );
    }

    template<class T>
    T *getInstance() const { return reinterpret_cast<T*>(m_ctx); }

private:
    void *m_ctx{nullptr};

    InitFuncType m_init{nullptr};
    ResetFuncType m_reset{nullptr};
    UpdateFuncType m_update{nullptr};
    RunFuncType m_run{nullptr};
};
    
} // namespace bldc

    
} // namespace motor

    
} // namespace pica


#endif /* !_PICA_DRIVE_MOTOR_CURRENT_CONTROLLER_HPP_ */
