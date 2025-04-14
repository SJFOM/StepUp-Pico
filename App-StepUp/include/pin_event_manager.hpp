/**
 * @file pin_event_manager.hpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2025-03-23
 *
 * @copyright Copyright (c) 2025
 * @license   MIT
 *
 */

#ifndef BUTTON_MANAGER_H_
#define BUTTON_MANAGER_H_

// pico-sdk
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`

// pin includes
#include "board_definitions.h"

// Logging utilities
#include "utils.h"

class PinEventManager
{
public:
    PinEventManager(uint8_t pin,
                    uint32_t event_type,
                    uint32_t long_press_duration_in_ms = 0U);
    ~PinEventManager();
    bool init();
    void deinit();

    void enableInterrupt(bool enable);

    void setDebounceTimerActive(bool active)
    {
        m_debounce_timer_active = active;
    }
    bool isDebounceTimerActive()
    {
        return m_debounce_timer_active;
    }

    void incrementPinEventOccured()
    {
        m_pin_event_occurred_count++;
    }
    void clearPinEventCount()
    {
        m_pin_event_occurred_count = 0;
    }
    uint32_t getPinEventCount()
    {
        return m_pin_event_occurred_count;
    }

    static int64_t pin_event_debounce_timer_callback(alarm_id_t id,
                                                     void *user_data);

    static const uint8_t s_max_pin_interrupt_count =
        31U;  // As per RP2040 datasheet, 31 pins with interrupt capability
    static PinEventManager *s_p_pin_event_manager[s_max_pin_interrupt_count];
    static uint8_t s_pin_interrupt_count;

    uint8_t m_pin;

    uint32_t m_event_type, m_pin_event_timeout_ms;

    alarm_id_t m_debounce_timer_id = 0;

    bool m_active_state_is_when_pin_is_high;  // True if the pin is active high,
    // false if low

protected:
private:
    uint32_t m_long_press_duration_in_ms;
    bool m_init_success;
    bool m_debounce_timer_active = false;

    uint32_t m_pin_event_occurred_count = 0;
};

#endif  // BUTTON_MANAGER_H_