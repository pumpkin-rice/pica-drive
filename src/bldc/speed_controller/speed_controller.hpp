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

#ifndef _PICA_DRIVE_BLDC_SPEED_CONTROLLER_HPP_
#define _PICA_DRIVE_BLDC_SPEED_CONTROLLER_HPP_

#include "hrt.h"
#include "utils/noncopyable.hpp"
#include <variant>
#include <cmath>

namespace pica
{

namespace motor
{

namespace bldc
{

class BLDC;

template<class Derived>
class SpeedController : Noncopyable
{
    friend class SpeedControllerProxy;
public:
    SpeedController(BLDC& motor) : m_motor(motor) {}

private:
    static void Reset(void *ctx)
    {
        reinterpret_cast<Derived*>(ctx)->reset();
    }

    static bool Init(void *ctx, void *cfg)
    {
        return reinterpret_cast<Derived*>(ctx)->init(cfg);
    }

    static bool Update(void *ctx, float *toque_ref, hrt_absnano now)
    {
        return reinterpret_cast<Derived*>(ctx)->update(toque_ref,now);
    }

protected:
    BLDC& m_motor;

    hrt_absnano m_ts_update{0};
};
    
} // namespace bldc

class SpeedControllerProxy : Noncopyable
{
public:
    typedef bool (*InitFuncType)(void *ctx, void *cfg);
    typedef void (*ResetFuncType)(void *ctx);
    typedef bool (*UpdateFuncType)(void *ctx, float *toque_ref, hrt_absnano now);

    template<class T>
    SpeedControllerProxy(T *current_controller)
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
    void reset(T *speed_controller)
    {
        assert(speed_controller != nullptr);

        m_ctx = speed_controller;

        m_reset = T::Reset;
        m_init = T::Init;
        m_update = T::Update;
    }

    bool init(void *cfg) { return m_init(m_ctx, cfg); }

    /**
     * @brief 重置控制器参数
     * 
     */
    void reset() { m_reset(m_ctx); }

    bool update(float *toque_ref, hrt_absnano now) { return m_update(m_ctx, toque_ref, now); }

private:
    void *m_ctx{nullptr};

    InitFuncType m_init;
    ResetFuncType m_reset;
    UpdateFuncType m_update;
};

}
    
} // namespace pica


#endif /* !_PICA_DRIVE_BLDC_SPEED_CONTROLLER_HPP_ */
