/**
 * @file voltage_monitoring.hpp
 * @author Sam (@SJFOM)
 * @brief Library for monitoring voltage levels and reporting on them when they
 * are detected to be out of bounds
 * @version 0.1
 * @date 2025-03-20
 *
 * @copyright Copyright (c) 2025
 * @license MIT
 */

#ifndef VOLTAGE_MONITORING_H_
#define VOLTAGE_MONITORING_H_

// Generic c includes
#include <string>

// pico-sdk
#include "hardware/adc.h"
#include "pico/stdlib.h"

// Control libraries
#include "../../../Interfaces/ControlInterface.hpp"

class VoltageMonitoring : public ControlInterface
{
public:
    VoltageMonitoring(const std::string &voltage_rail_name,
                      uint8_t voltage_pin,
                      uint8_t voltage_adc_channel,
                      float voltage_scaling_factor,
                      float voltage_threshold_low,
                      float voltage_threshold_high);
    ~VoltageMonitoring();

    bool init() override;
    void deinit() override;
    enum ControllerState processJob(uint32_t tick_count) override;

    float getVoltage() const;

protected:
private:
    const std::string m_voltage_rail_name;
    uint8_t m_voltage_pin, m_voltage_adc_channel;
    float m_voltage_scaling_factor, m_voltage_threshold_low,
        m_voltage_threshold_high;

    bool m_init_success;
    float m_current_voltage;

    void updateVoltageRead();
};

#endif  // VOLTAGE_MONITORING_H_