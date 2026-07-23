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

#include "drive_conf.h"
#include "motor/bldc.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>

using namespace pica::motor;

class BLDCFixture : public ::testing::Test
{
public:
    void SetUp() override
    {
        initMotorConfig();
        
        m_bldc.init(&m_cfg);
    }

    void TearDown() override
    {

    }

    void sample();

    void freshOutput();

    void constTorque();

private:
    void initMotorConfig();

protected:
    Config m_cfg;
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

void BLDCFixture::initMotorConfig()
{
    m_cfg.initDefaultValue();

    auto& speed_cfg = m_cfg.speed_controller_cfg;

    m_cfg.motor_type = BLDC::HIGH_CURRENT;
    m_cfg.current_controller_type = BLDC::CurrentControllerType::FieldOrientedControl,

    m_cfg.pole_pairs        = 5;
    m_cfg.phase_inductance  = 0.5 * 0.64e-3,
    m_cfg.phase_resistance  = 0.5 * 0.57,
    m_cfg.shunt_conductance = 1/50e-3; // 50mR
    m_cfg.torque_constant   = 0.0591758042f;
    m_cfg.current_limit     = 7.81;
    m_cfg.inertia = 0.0000177245;

    // 时间常数
    float Tq = m_cfg.phase_inductance / m_cfg.phase_resistance;
    float Td = m_cfg.phase_inductance / m_cfg.phase_resistance;

    m_cfg.current_controller_bandwidth = 2 * M_PI / fminf(Tq, Td);

    float speed_bw = 3871/60 * 2*M_PI; // 速度环带宽：4000 rpm

    speed_cfg.control_mode = pica::motor::SpeedController::kVelocity;
    speed_cfg.pi.pos_gain = 1.f;
    speed_cfg.pi.vel_gain = (speed_bw * m_cfg.inertia) / m_cfg.torque_constant;
    speed_cfg.pi.vel_integrator_gain = speed_bw * speed_cfg.pi.vel_gain;
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

TEST_F(BLDCFixture, ConstTorque)
{
    float delta_t = 1e-3;
    float torque = 0.1f;

    for (int i = 0; i < 1000*10; i++)
    {
        double ts_diff = m_ts_now - m_ts_last;

        this->sample();

        m_bldc.sampleBusVoltageHandler(m_vbus);
        m_bldc.sampleEncoderHandler(m_theta_mach, m_omega_mach);
        m_bldc.sampleCurrentHandler(m_voltage_shunt, (hrt_absnano)(m_ts_now * 1e9));
        
        m_bldc.do_checks();

        m_bldc.setVelocity(m_speed);

        m_bldc.update(ts_diff);

        m_bldc.sampleCurrentCalibratorHandler(NULL, PICA_DRIVE_CURRENT_MEASURE_PERIOD);

        m_bldc.run((hrt_absnano)(m_ts_now * 1e9));

        this->freshOutput();
    }
}
