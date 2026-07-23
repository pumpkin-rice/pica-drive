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

#include "current_controller.hpp"

namespace pica::motor::bldc
{

constexpr float kFOCIDQMeasFilterKDefault = 1.f;

class FOC : public CurrentController<FOC>
{
    friend class CurrentControllerVariant;
    friend class CurrentControllerRefVariant;
    friend CurrentController<FOC>;

public:
    struct Config
    {
        float pos_gain = 20.f; /*!< (turn/s)/turn */
        float vel_gain = 1.f/6.f; /*!< Nm/(turn/s) */
        float vel_integrator_gain = 2.0f/6.0f;
    };

    FOC(BLDC& motor) : CurrentController(motor) {}

    /**
     * @brief 更新 PI 控制器增益
     * 
     */
    void updateGain();

    const AlphaBeta& getCurrentAlphaBetaMeasured() const
    {
        return m_i_alpha_beta_meas;
    }
    const DQ& getCurrentDQMeasured() const
    {
        return m_idq_meas;
    }

    const DQ& getVoltageDQSetpoint() const
    {
        return m_vdq_sp;
    }

    const DQ& getVoltageDQFinal() const
    {
        return m_vdq_final;
    }

    void reset();

    bool init(void *cfg);

    bool update(hrt_absnano now);

    bool run(hrt_absnano ts_output, AlphaBeta *v_alpha_beta_final);

#if (PICA_DRIVE_ENABLE_DEBUG == 1)

    const auto& getControllerParam() const
    {
        return m_pi;
    }

#endif

private:
    Config m_cfg;

    DQ m_vdq_sp;
    DQ m_idq_sp;

    AlphaBeta m_i_alpha_beta_meas;
    DQ m_idq_meas;
    DQ m_vdq_final;

    float m_idq_meas_filter_k; /*!< dq 电流滤波器参数 */

    struct {
        DQ kp;
        DQ ki;
        DQ integral;
        DQ err_prev;
    } m_pi;
};

} // namespace pica::motor

#endif /* !_PICA_DRIVE_BLDC_FOC_HPP_ */
