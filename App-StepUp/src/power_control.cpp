/**
 * @file power_control.cpp
 * @author Sam (@SJFOM)
 * @brief Implementation of the PowerControl class
 * @details ...
 * @version 0.1
 * @date 2025-04-17
 *
 * @copyright Copyright (c) 2025
 * @license MIT
 */

#include "../include/power_control.hpp"
#include "PicoUtils.h"

PowerControl::PowerControl(uint32_t power_button_hold_timeout_ms,
                           uint32_t power_down_inactive_timeout_ms)
    : m_power_button_hold_timeout_ms(power_button_hold_timeout_ms),
      m_power_down_inactive_timeout_ms(power_down_inactive_timeout_ms)
{
}

PowerControl::~PowerControl()
{
    deinit();
}

bool PowerControl::init()
{
    m_init_success = true;

    // Enable power control control pins
    gpio_init(MCU_PWR_CTRL_PIN);
    gpio_set_dir(MCU_PWR_CTRL_PIN, GPIO_OUT);

    gpio_init(MCU_PWR_BTN_PIN);
    gpio_set_dir(MCU_PWR_BTN_PIN, GPIO_IN);
    gpio_pull_up(MCU_PWR_BTN_PIN);

    // Set up usb monitoring pin
    gpio_set_input_enabled(VUSB_MONITOR_PIN, true);
    gpio_disable_pulls(VUSB_MONITOR_PIN);

    // Assert power control pin HIGH to keep circuit powered
    gpio_put(MCU_PWR_CTRL_PIN, 1);

    // TODO: Implement detection of both falling and rising edges
    m_usb_pin_event_manager =
        new PinEventManager(VUSB_MONITOR_PIN, GPIO_IRQ_EDGE_FALL);

    m_power_pin_event_manager =
        new PinEventManager(MCU_PWR_BTN_PIN,
                            GPIO_IRQ_EDGE_FALL,
                            m_power_button_hold_timeout_ms);

    m_init_success |= m_usb_pin_event_manager->init();
    m_init_success |= m_power_pin_event_manager->init();

    m_power_down_triggered = false;

    return m_init_success;
}

void PowerControl::deinit()
{
    m_usb_pin_event_manager->deinit();
    m_power_pin_event_manager->deinit();
    delete m_usb_pin_event_manager, m_power_pin_event_manager;
}

bool PowerControl::isUSBInserted()
{
    return m_is_usb_inserted;
}

bool PowerControl::isPowerDownTriggered()
{
    return m_power_down_triggered;
}

int64_t power_down_timer_callback(alarm_id_t id, void *user_data)
{
    // FIXME: These pins should be handled in their respective classes vs having
    // inherent knowledge of them here

    // Power down the TMC
    gpio_put(TMC_PIN_ENABLE, 0);

    // Power down the boost converter
    gpio_put(TMC_PIN_BOOST_EN, 0);

    // Power down the MCU
    gpio_put(MCU_PWR_CTRL_PIN, 0);

    while (1);
    return 0;
}

void PowerControl::triggerPowerDownProcess()
{
    m_power_down_triggered = true;

    // Start a timer callback to enable powering down the rest of the
    // circuit in main.cpp and, then, physcially power off
    add_alarm_in_ms(500U,
                    power_down_timer_callback,
                    NULL, /*user_data*/
                    false /*fire_if_past*/);
}

enum ControllerState PowerControl::processJob(uint32_t tick_count)
{
    enum ControllerState power_state = ControllerState::STATE_IDLE;

    if (!m_init_success)
    {
        LOG_ERROR("PowerControl not initialized.");
        return STATE_IDLE;
    }

    uint32_t last_activate_timestamp =
        ControlInterface::getLastTimeControlPeripheralActivityWasUpdatedMs();

    bool power_down_timestamp_elapsed =
        PicoUtils::getCurrentTimestampMs() - last_activate_timestamp >
        m_power_down_inactive_timeout_ms;

    if (m_usb_pin_event_manager->hasEventOccurred())
    {
        m_is_usb_inserted = true;
        LOG_INFO("USB cable removed");
        m_usb_pin_event_manager->clearPinEventCount();
        power_state = ControllerState::STATE_NEW_DATA;
    }
    else if (m_power_pin_event_manager->hasEventOccurred() ||
             power_down_timestamp_elapsed)
    {
        LOG_INFO("Power down event triggered");

        m_power_pin_event_manager->clearPinEventCount();
        power_state = ControllerState::STATE_NEW_DATA;

        triggerPowerDownProcess();
    }

    return power_state;
}
