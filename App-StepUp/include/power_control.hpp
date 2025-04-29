/**
 * @file power_control.hpp
 * @author Sam (@SJFOM)
 * @brief Library for monitoring voltage levels and reporting on them when they
 * are detected to be out of bounds
 * @version 0.1
 * @date 2025-04-17
 *
 * @copyright Copyright (c) 2025
 * @license MIT
 */

#ifndef POWER_CONTROL_H_
#define POWER_CONTROL_H_

// Generic c includes
#include <string>

// pico-sdk
#include "hardware/adc.h"
#include "pico/stdlib.h"

// Control libraries
#include "ControlInterface.hpp"

// PinManager
#include "pin_event_manager.hpp"

class PowerControl : public ControlInterface
{
public:
    PowerControl(
        uint32_t power_button_hold_timeout_ms,
        uint32_t power_down_timeout_inactive_ms = 600000U /* 10 minutes */);
    ~PowerControl();

    bool init() override;
    void deinit() override;
    enum ControllerState processJob(uint32_t tick_count) override;

    bool isUSBInserted();

    bool isPowerDownTriggered();

    void triggerPowerDownProcess();

protected:
private:
    PinEventManager *m_usb_pin_event_manager, *m_power_pin_event_manager;

    uint32_t m_power_button_hold_timeout_ms, m_power_down_inactive_timeout_ms;
    bool m_is_usb_inserted, m_power_down_triggered;

    enum ControllerNotification m_power_control_notify;
};

#endif  // POWER_CONTROL_H_