/**
 * @file config_manager.hpp
 * @author Pumpkin Rice
 * @brief 
 * @version 0.1
 * @date 2026-07-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_BLDC_CONFIG_MANAGER_HPP_
#define _PICA_DRIVE_BLDC_CONFIG_MANAGER_HPP_

#include "bldc/bldc.hpp"
#include "utils/noncopyable.hpp"
#include "utils/singleton.hpp"

namespace pica
{

namespace motor
{

namespace bldc
{

class ConfigManager : Noncopyable
{
public:
    ConfigManager() = default;
    ConfigManager(void *cfg) : m_cfg(reinterpret_cast<Flash*>(cfg)) {}
    
    /**
     * @brief Flash 中的配置文件排列
     * 
     */
    struct Flash {
        BLDC::Config motor;

        struct {
            SpeedControllerPI::Config pi;
        } speed; /*!< 速度控制器参数 */

        struct {
            FOC::Config foc;
        } current; /*!< 电流控制器配置参数 */
    };

    BLDC::Config *motor() { return &m_cfg->motor; }
    SpeedControllerPI::Config *speed() { return &m_cfg->speed.pi; }
    FOC::Config *current() { return &m_cfg->current.foc; }

    bool write();
    bool flush();

    void init(Flash *addr) { m_cfg = addr; }

private:

private:
    Flash *m_cfg{nullptr};
};

using ConfigMgr = Singleton<ConfigManager>;
    
} // namespace bldc

    
} // namespace motor

    
} // namespace pica


#endif /* !_PICA_DRIVE_BLDC_CONFIG_MANAGER_HPP_*/
