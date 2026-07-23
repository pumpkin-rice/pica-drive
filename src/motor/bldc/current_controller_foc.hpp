/**
 * @file foc.hpp
 * @author Pumpkin Rice
 * @brief
 * @version 0.1
 * @date 2026-06-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef _PICA_DRIVE_BLDC_FOC_HPP_
#define _PICA_DRIVE_BLDC_FOC_HPP_

#include "motor/current_controller.hpp"
#include "motor/park/park.h"

namespace pica::motor
{

constexpr float kFOCIDQMeasFilterKDefault = 1.f;

class FOC : public CurrentController
{
public:
    struct ControllerPI;

    FOC(Motor& motor) : CurrentController(motor) {}

    void updateGain();
    void reset();

    bool init(const Config *cfg) override;

    bool update(float torque_sp, hrt_absnano now) final;

    CurrentController::ControllerLoopFuncType getControllerLoopFunc() final
    {
        return Run;
    }

    const AlphaBeta& getCurrentAlphaBetaMeasured() const
    {
        return m_i_alpha_beta_meas;
    }
    const DQ& getCurrentDQMeasured() const
    {
        return m_idq_meas;
    }
    const DQ& getVoltageDQFinal() const
    {
        return m_vdq_final;
    }
    const DQ& getVoltageDQSetpoint() const
    {
        return m_vdq_sp;
    }
    const AlphaBeta& getVoltageAlphaBetaFinal() const
    {
        return m_v_alpha_beta_final;
    }

    const ControllerPI& getCurrentControllerD() const { return m_pi_id; }
    const ControllerPI& getCurrentControllerQ() const { return m_pi_iq; }

private:
    /**
     * @brief 运行电流环
     * 
     * @param[in] ctrl 
     * @param[in] ts_next_pwmoutput 下一次 PWM 输出时刻
     * @return true 
     * @return false 
     */
    static bool Run(CurrentController *ctrl, hrt_absnano ts_next_pwmoutput);

private:
    DQ m_vdq_sp;
    DQ m_idq_sp;

    AlphaBeta m_i_alpha_beta_meas;
    DQ m_idq_meas;
    DQ m_vdq_final;
    AlphaBeta m_v_alpha_beta_final;

    float m_idq_meas_filter_k; /*!< dq 电流滤波器参数 */

    hrt_absnano m_update_ts{0}; /*!< 更新时刻, ns */

    struct ControllerPI
    {
        float kp;
        float ki, integral, integral_limit;
        float output_limit;

        float err_prev, output_prev;
    } m_pi_id, m_pi_iq;
};

} // namespace pica::motor

#endif /* !_PICA_DRIVE_BLDC_FOC_HPP_ */
