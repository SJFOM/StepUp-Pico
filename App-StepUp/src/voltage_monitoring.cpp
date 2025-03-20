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
#include "../include/utils.h"  // For logging utilities

VoltageMonitoring::VoltageMonitoring(const std::string &voltage_rail_name,
                                     uint8_t voltage_pin,
                                     uint8_t voltage_adc_channel,
                                     float voltage_scaling_factor,
                                     float voltage_threshold_low,
                                     float voltage_threshold_high)
    : m_voltage_rail_name(voltage_rail_name),
      m_voltage_pin(voltage_pin),
      m_voltage_adc_channel(voltage_adc_channel),
      m_voltage_scaling_factor(voltage_scaling_factor),
      m_voltage_threshold_low(voltage_threshold_low),
      m_voltage_threshold_high(voltage_threshold_high),
      m_init_success(false),
      m_current_voltage(0.0f)
{
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
             m_current_voltage);

    if (false == Utils::isNumberWithinBounds<float>(m_current_voltage,
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
    if (!m_init_success)
    {
        LOG_WARN("VoltageMonitoring not initialized.");
        return STATE_IDLE;
    }

    // Read the current voltage
    updateVoltageRead();

    // Determine the state based on voltage thresholds
    if (false == Utils::isNumberWithinBounds<float>(m_current_voltage,
                                                    m_voltage_threshold_low,
                                                    m_voltage_threshold_high))
    {
        return ControllerState::STATE_NEW_DATA;
    }

    return STATE_READY;
}

float VoltageMonitoring::getVoltage() const
{
    return m_current_voltage;
}

void VoltageMonitoring::updateVoltageRead()
{
    // TODO: Consider using a moving average to smooth the voltage reading
    m_current_voltage = Utils::getValidADCResultVolts(m_voltage_adc_channel);
    m_current_voltage *= m_voltage_scaling_factor;
}