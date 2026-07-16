/**
 * @file test_bldc.cc
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-06-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bldc/bldc.hpp"
#include "bldc/config_manager.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>

using namespace pica::motor::bldc;

#include "hrt.h"
#include "utils/singleton.hpp"

class AbsoultTime
{
public:
    /**
     * @brief Construct a new Absoult Time object
     * 
     * @param[in] tick2sec 
     */
    AbsoultTime(double tick2sec) : m_tick2sec(tick2sec) {}

    AbsoultTime() = default;

    hrt_absnano update() { return ++m_now; }

    hrt_absnano tick() const { return m_now; }
    double second() const { return m_now * m_tick2sec; }
    double period() const { return m_tick2sec; }

private:
    hrt_absnano m_now{0};
    double m_tick2sec{1/48e3};
};

using AbsoultTimeSingle = pica::Singleton<AbsoultTime>;

class BLDCFixture : public ::testing::Test
{
public:
    void SetUp() override;

    void TearDown() override
    {

    }

    void initConfig();

    void sample();

    void freshOutput();

    void constTorque();

protected:
    ConfigManager::ConfigFlash m_cfg;
    BLDC m_bldc;

    uint64_t m_tick{0};
    double m_second_per_tick{0.f}; /*!< s/tick */

    double m_ts_now{0};
    double m_ts_last{0};

    float m_freq{0.f};

    float m_voltage_shunt[3], m_ibus, m_voltage_shunt_calibrator[3];
    float m_voltage_phase[3], m_vbus;
    float m_duty_cycles[3];

    float m_theta_mach; /*!< 电机机械角度, rad */
    float m_omega_mach; /*!< 电机机械角速度, rad/s */

    float m_torque{0};
    float m_speed{0};
    float m_position{0};
};

void BLDCFixture::SetUp()
{
    ConfigMgr::GetInstance()->init(&m_cfg);

    initConfig();
    
    m_bldc.init();
}

void BLDCFixture::sample()
{
        // 更新时间
    ++m_tick;
    m_ts_last = m_ts_now;
    m_ts_now = m_tick * m_second_per_tick;

    m_vbus = 24;

    float angle = m_ts_now * m_freq;

    std::array<float, 3> voltage_shunt = {
        std::cosf(angle),
        std::cosf(angle + 2./3.*M_PI),
        std::cosf(angle + 4./3.*M_PI),
    };

    // m_voltage_shunt[0] = (voltage_shunt / m_motor_param.shunt_conductance) / m_bldc.parent.phase_current_rev_gain;
    // m_voltage_shunt[1] = (voltage_shunt / m_motor_param.shunt_conductance) / m_bldc.parent.phase_current_rev_gain;
    // m_voltage_shunt[2] = (voltage_shunt / m_motor_param.shunt_conductance) / m_bldc.parent.phase_current_rev_gain;

    // 电流电压
    m_voltage_shunt[0] = voltage_shunt[0];
    m_voltage_shunt[1] = voltage_shunt[1];
    m_voltage_shunt[2] = voltage_shunt[0];

    m_voltage_shunt_calibrator[0] = 0.f;
    m_voltage_shunt_calibrator[1] = 0.f;
    m_voltage_shunt_calibrator[2] = 0.f;

    m_theta_mach = angle;
    m_omega_mach = m_freq;

    // 输入目标值
    m_torque   = 0.0f;
    m_speed    = 1200/60*(2*M_PI);
    m_position = 0.f;
}

void BLDCFixture::initConfig()
{
    auto& motor = m_cfg.motor;

    motor.motor_type = BLDC::kHighCurrent;
    motor.current_controller_type = BLDC::CurrentControllerType::kFOC,

    motor.pole_pairs        = 5;
    motor.phase_inductance  = 0.5 * 0.64e-3,
    motor.phase_resistance  = 0.5 * 0.57,
    motor.shunt_conductance = 1/50e-3; // 50mR
    motor.torque_constant   = 0.0591758042f;
    motor.current_limit     = 7.81;

    // 时间常数
    float Tq = motor.phase_inductance / motor.phase_resistance;
    float Td = motor.phase_inductance / motor.phase_resistance;

    motor.current_controller_bandwidth = 2 * M_PI / fminf(Tq, Td);
}

void BLDCFixture::freshOutput()
{
    //   [iabc, ialpha_beta, idq, vdq, v_alpha_beta_final, duty_cycle] = mex_picadrive_foc_current_loop(ts, theta_elec, omega_elec, iabc, target);
    // bldc_get_current_controller_foc(&m_bldc);
    
    // const float *iabc = bldc_get_current_phase_meas(&m_bldc);
    // const float *i_alpha_beta_meas = foc_dbg_i_alpha_beta_measured(foc);
    // const float *idq_meas = foc_dbg_idq_meas(foc);
    // const float *vdq = foc_dbg_vdq(foc);
    // const float *v_alpha_beta_final = foc_dbg_v_alpha_beta_final(foc);
    // const float *duty_cycle = bldc_get_duty_cycle(&m_bldc);
}

TEST_F(BLDCFixture, SpeedPI)
{
    float delta_t = 1e-3;
    float torque = 0.1f;

    for (int i = 0; i < 1000*10; i++)
    {
        double ts_diff = m_ts_now - m_ts_last;

        this->sample();

        m_bldc.sampleBusVoltageHandler(m_vbus);
        m_bldc.sampleEncoderHandler(m_theta_mach, m_omega_mach);
        m_bldc.sampleCurrentHandler(m_voltage_shunt, m_tick);
        
        m_bldc.do_checks();

        m_bldc.setVelocitySetPoint(m_speed);

        m_bldc.update(ts_diff);

        m_bldc.sampleCurrentCalibratorHandler(NULL, ts_diff);

        m_bldc.run(m_tick);

        this->freshOutput();
    }
}
