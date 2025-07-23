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
#include "ControlInterface.hpp"

enum class VoltageBoundsCheckState
{
    VOLTAGE_STATE_WITHIN_BOUNDS = 0U,
    VOLTAGE_STATE_OUTSIDE_BOUNDS = 1U,
    VOLTAGE_STATE_MAX_COUNT,
};

enum class VoltageBoundsSensitivityState
{
    VOLTAGE_STATE_SENSITIVITY_LOW = 0U,
    VOLTAGE_STATE_SENSITIVITY_MEDIUM = 1U,
    VOLTAGE_STATE_SENSITIVITY_HIGH = 2U,
    VOLTAGE_STATE_SENSITIVITY_INSTANT = 3U,
    VOLTAGE_STATE_SENSITIVITY_MAX_COUNT,
};

struct VoltageMonitorData
{
    float voltage;
    VoltageBoundsCheckState state;
};

class VoltageMonitoring : public ControlInterface
{
public:
    VoltageMonitoring(
        const std::string &voltage_rail_name,
        uint8_t voltage_pin,
        uint8_t voltage_adc_channel,
        float voltage_scaling_factor,
        float voltage_threshold_low,
        float voltage_threshold_high,
        float voltage_delta_threshold,
        VoltageBoundsSensitivityState sensitivity_state =
            VoltageBoundsSensitivityState::VOLTAGE_STATE_SENSITIVITY_MEDIUM);
    ~VoltageMonitoring();

    bool init() override;
    void deinit() override;
    enum ControllerState processJob(uint32_t tick_count) override;

    struct VoltageMonitorData getVoltageData() const;

    void setVoltageThresholds(float voltage_threshold_low,
                              float voltage_threshold_high);

protected:
private:
    const std::string m_voltage_rail_name;
    uint8_t m_voltage_pin, m_voltage_adc_channel;
    float m_voltage_scaling_factor, m_voltage_threshold_low,
        m_voltage_threshold_high, m_voltage_delta_threshold;

    VoltageBoundsSensitivityState m_sensitivity_state;

    float m_latest_voltage;

    struct VoltageMonitorData m_voltage_data;

    Utils::ExponentialMovingAverage m_voltage_average;

    void updateVoltageReadMovingAverage();
};

#endif  // VOLTAGE_MONITORING_H_