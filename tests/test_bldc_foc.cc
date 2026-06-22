/**
 * @file mex_ipark.cc
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#define ENABLE_PICA_DRIVE_DEBUG 1
#include "bldc.h"

#include "utils/noncopyable.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <array>

class BLDC : pica::Noncopyable {
public:
    BLDC() = delete;

    BLDC(double delta_t, float rpm)
        : m_second_per_tick(delta_t)
        , m_freq(rpm / 60. * 2. * M_PI) {}
    ~BLDC() = default;

    /**
     * @brief 初始化
     * 
     */
    void init();

    /**
     * @brief 数据采样（生成模拟数据）
     * 
     */
    void sample();

    /**
     * @brief 运行控制程序
     * 
     * @param[in] ts 
     * @param[in] angle 
     * @param[in] omega 
     * @param[in] current 
     */
    void run();

    void fresh_output();

private:
    void log();

    double diff_timestamp(const struct timespec *start, const struct timespec *end)
    {
        return (end->tv_sec - start->tv_sec) + (end->tv_nsec - start->tv_nsec) * 1e-9;
    }

    int getTimestamp(struct timespec *ts)
    {
        struct timeval tv;

        if (gettimeofday(&tv, NULL) < 0) {
            return -1;
        }

        ts->tv_sec = tv.tv_sec;
        ts->tv_nsec = tv.tv_usec * 1000;

        return 0;
    }

private:
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


    struct bldc m_bldc;
    struct motor_parameters m_motor_param;
};

void BLDC::run()
{
    double ts_diff = m_ts_now - m_ts_last;

    // 更新传感器参数
    bldc_sample_voltage_handler(&m_bldc, NULL, m_vbus);
    bldc_sample_encoder_handler(&m_bldc, m_theta_mach, m_omega_mach);

    // 更新采样电流
    bldc_sample_current_handler(&m_bldc, m_voltage_shunt);

    bldc_do_checks(&m_bldc);

    // 更新电机控制环参数
    bldc_set_torque(&m_bldc, m_torque);

    bldc_update(&m_bldc);

    // 更新校正电流
    bldc_sample_current_calibrator_handler(&m_bldc, NULL, ts_diff);
    
    // 运行电流环
    bldc_run(&m_bldc, ts_diff, ts_diff);
}

void BLDC::init()
{
    motor_init_param_by_default(&m_motor_param);
    
    m_motor_param.type = MOTOR_TYPE_HIGH_CURRENT,
    m_motor_param.ctrl_type = MOTOR_CTRL_TYPE_FOC,

    m_motor_param.pole_pairs        = 5;
    m_motor_param.phase_inductance  = 0.5 * 0.64e-3,
    m_motor_param.phase_resistance  = 0.5 * 0.57,
    m_motor_param.shunt_conductance = 1/50e-3; // 50mR
    m_motor_param.torque_constant   = 0.0591758042f;
    m_motor_param.current_limit     = 7.81;

    struct motor_parameters *params = &m_motor_param;

    // 时间常数
    float Tq = params->phase_inductance / params->phase_resistance;
    float Td = params->phase_inductance / params->phase_resistance;

    params->current_controller_bandwidth = 2 * M_PI / fminf(Tq, Td);

    bldc_init(&m_bldc, &m_motor_param);
}

void BLDC::sample()
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
    m_torque   = 0.2f;
    m_speed    = 0.f;
    m_position = 0.f;
}

void BLDC::fresh_output()
{
    //   [iabc, ialpha_beta, idq, vdq, v_alpha_beta_final, duty_cycle] = mex_picadrive_foc_current_loop(ts, theta_elec, omega_elec, iabc, target);
    struct foc *foc = bldc_get_current_controller_foc(&m_bldc);
    
    const float *iabc = bldc_get_current_phase_meas(&m_bldc);
    const float *i_alpha_beta_meas = foc_dbg_i_alpha_beta_measured(foc);
    const float *idq_meas = foc_dbg_idq_meas(foc);
    const float *vdq = foc_dbg_vdq(foc);
    const float *v_alpha_beta_final = foc_dbg_v_alpha_beta_final(foc);
    const float *duty_cycle = bldc_get_duty_cycle(&m_bldc);
}

int main(int argc, char const *argv[])
{
    BLDC bldc(1e-3, 1800);

    bldc.init();

    for (;;) {
        bldc.sample();
        bldc.run();
        bldc.fresh_output();
    }

    return 0;
}

