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

PowerControl::PowerControl(uint32_t power_down_timeout_in_ms)
    : m_power_down_timeout_in_ms(power_down_timeout_in_ms)
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
        new PinEventManager(MCU_PWR_CTRL_PIN, GPIO_IRQ_EDGE_FALL, 5000U);

    m_init_success |= m_usb_pin_event_manager->init();
    m_init_success |= m_power_pin_event_manager->init();

    return m_init_success;
}

void PowerControl::deinit()
{
    m_usb_pin_event_manager->deinit();
    m_power_pin_event_manager->deinit();
    delete m_usb_pin_event_manager, m_power_pin_event_manager;
}

void PowerControl::powerDown()
{
    LOG_INFO("Power button pressed");

    // Set INFO notification status
    enum ControllerNotification usb_detect_notify =
        ControllerNotification::NOTIFY_POWER_DOWN;

    // TODO: Hand over this functionality to other running processes

    // TODO: Power down the boost converter
    gpio_put(TMC_PIN_BOOST_EN, 0);
    // TODO: Safely power down the TM2300
    // tmc_control.enableFunctionality(false);
    // TODO: Change the LED pattern to indicate power down
    // Power down the circuit
    gpio_put(MCU_PWR_CTRL_PIN, 0);

    // FIXME: This is a blocking call to wait for the power down to
    // complete
    while (1);
}

bool PowerControl::isUSBInserted()
{
    return m_is_usb_inserted;
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
        ControlInterface::getLastTimeControlPeripheralWasUsedMs();

    LOG_DATA("Last activated timestamp: %d", last_activate_timestamp);

    LOG_DATA("Time since last activate: %lu",
             Utils::getCurrentTimestampMs() - last_activate_timestamp);

    bool power_down_timestamp_elapsed =
        Utils::getCurrentTimestampMs() - last_activate_timestamp >
        m_power_down_timeout_in_ms;

    if (m_usb_pin_event_manager->hasEventOccurred())
    {
        m_is_usb_inserted = true;
        LOG_INFO("USB cable detected");
        m_usb_pin_event_manager->clearPinEventCount();
        power_state = ControllerState::STATE_NEW_DATA;
    }

    if (m_power_pin_event_manager->hasEventOccurred() ||
        power_down_timestamp_elapsed)
    {
        m_power_pin_event_manager->clearPinEventCount();
        power_state = ControllerState::STATE_NEW_DATA;

        // FIXME: Implement a timeout for this to allow time for other loops to
        // power down.
        powerDown();
    }

    return power_state;
}
