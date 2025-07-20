/**
 * @file voltage_monitoring.cpp
 * @brief Implementation of the VoltageMonitoring class
 * @details Library for monitoring voltage levels and reporting on them when
 * they are detected to be out of bounds
 * @version 0.1
 * @date 2025-03-20
 *
 * @copyright Copyright (c) 2025
 * @license MIT
 */

#include "../include/voltage_monitoring.hpp"
#include "PicoUtils.h"

VoltageMonitoring::VoltageMonitoring(const std::string &voltage_rail_name,
                                     uint8_t voltage_pin,
                                     uint8_t voltage_adc_channel,
                                     float voltage_scaling_factor,
                                     float voltage_threshold_low,
                                     float voltage_threshold_high,
                                     float voltage_delta_threshold)
    : m_voltage_rail_name(voltage_rail_name),
      m_voltage_pin(voltage_pin),
      m_voltage_adc_channel(voltage_adc_channel),
      m_voltage_scaling_factor(voltage_scaling_factor),
      m_voltage_threshold_low(voltage_threshold_low),
      m_voltage_threshold_high(voltage_threshold_high),
      m_voltage_delta_threshold(voltage_delta_threshold)
{
    m_voltage_data.voltage = 0.0f;
    m_voltage_data.state = VoltageBoundsCheckState::VOLTAGE_STATE_WITHIN_BOUNDS;

    m_voltage_average =
        Utils::ExponentialMovingAverage(csx_exponential_moving_average_alpha);
}

VoltageMonitoring::~VoltageMonitoring()
{
    deinit();
}

bool VoltageMonitoring::init()
{
    m_init_success = true;

    // Initialize ADC hardware
    adc_init();
    adc_gpio_init(m_voltage_pin);

    // Give time for the voltage on the boost converter ADC pin to settle
    sleep_ms(100);

    updateVoltageRead();

    LOG_DATA("%s voltage: %.2fV",
             m_voltage_rail_name.c_str(),
             m_voltage_data.voltage);

    if (false == Utils::isNumberWithinBounds<float>(m_voltage_data.voltage,
                                                    m_voltage_threshold_low,
                                                    m_voltage_threshold_high))
    {
        m_init_success = false;
        LOG_ERROR("Voltage out of range... FAIL");
    }

    return m_init_success;
}

void VoltageMonitoring::deinit()
{
    // Deinitialize ADC or any other resources if needed
    m_init_success = false;
    LOG_INFO("VoltageMonitoring deinitialized.");
}

enum ControllerState VoltageMonitoring::processJob(uint32_t tick_count)
{
    float s_last_voltage = m_voltage_data.voltage;
    enum ControllerState monitor_state = ControllerState::STATE_IDLE;

    if (!m_init_success)
    {
        LOG_ERROR("VoltageMonitoring not initialized.");
        return STATE_IDLE;
    }

    // Read the current voltage - updates m_voltage_data.voltage
    updateVoltageRead();

    // Determine the state based on voltage thresholds
    if (false == Utils::isNumberWithinBounds<float>(m_voltage_data.voltage,
                                                    m_voltage_threshold_low,
                                                    m_voltage_threshold_high))
    {
        m_voltage_data.state =
            VoltageBoundsCheckState::VOLTAGE_STATE_OUTSIDE_BOUNDS;
        monitor_state = ControllerState::STATE_NEW_DATA;
    }
    else
    {
        // Important to set/reset this value if it was previously out of bounds
        m_voltage_data.state =
            VoltageBoundsCheckState::VOLTAGE_STATE_WITHIN_BOUNDS;
    }

    // New state info if the voltage has changed by more than
    // m_voltage_delta_threshold
    if (Utils::isNumberWithinBounds<float>(
            m_voltage_data.voltage,
            s_last_voltage - m_voltage_delta_threshold,
            s_last_voltage + m_voltage_delta_threshold))
    {
        monitor_state = ControllerState::STATE_NEW_DATA;
    }

    return monitor_state;
}

void VoltageMonitoring::setVoltageThresholds(float voltage_threshold_low,
                                             float voltage_threshold_high)
{
    assert(voltage_threshold_low >= 0 &&
           voltage_threshold_high > voltage_threshold_low);
    m_voltage_threshold_low = voltage_threshold_low;
    m_voltage_threshold_high = voltage_threshold_high;
}

struct VoltageMonitorData VoltageMonitoring::getVoltageData() const
{
    return m_voltage_data;
}

void VoltageMonitoring::updateVoltageRead()
{
    float latest_voltage_in_volts =
        PicoUtils::getValidADCResultVolts(m_voltage_adc_channel) *
        m_voltage_scaling_factor;

    // Add latest voltage read value to the exponential moving average
    m_voltage_average.push(latest_voltage_in_volts);

    // Return latest average voltage value
    m_voltage_data.voltage = m_voltage_average.getAverage();
}