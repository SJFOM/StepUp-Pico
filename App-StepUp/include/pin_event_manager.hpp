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
#include "PicoUtils.h"

constexpr static uint32_t cxs_pin_debounce_default_delay_time_ms = 50U;
constexpr static uint32_t cxs_adc_settling_default_time_between_reads_in_ms =
    50U;

class PinEventManager
{
public:
    PinEventManager(uint8_t pin,
                    uint32_t event_type,
                    uint32_t long_press_duration_in_ms = 0U);
    ~PinEventManager();
    bool init();
    void deinit();

    void setDebounceTimerActive(bool active);
    bool isDebounceTimerActive() const;

    void incrementPinEventOccured();
    void clearPinEventCount();
    uint32_t getPinEventCount() const;

    bool hasEventOccurred() const;

    inline void setDebounceTimerId(alarm_id_t id);
    inline alarm_id_t getDebounceTimerId() const;

    static int64_t pin_event_debounce_timer_callback(alarm_id_t id,
                                                     void *user_data);

    // As per RP2040 datasheet, 31 pins with interrupt capability. Failing
    // strong requirements, this library offers up to two PinEventManager
    // instances for the same pin: 2 button press durations for different kinds
    // of application functionality
    static const uint8_t s_max_pin_interrupt_count = 62U;
    static PinEventManager *s_p_pin_event_manager[s_max_pin_interrupt_count];
    static uint8_t s_pin_event_manager_instance_count;

    uint8_t m_pin;
    uint32_t m_pin_event_type, m_pin_event_timeout_ms;

    bool m_active_state_is_when_pin_is_high;  // True if the pin is active high,
    // false if low

protected:
private:
    uint32_t m_long_press_duration_in_ms;
    bool m_init_success;
    bool m_debounce_timer_active = false;

    uint32_t m_pin_event_occurred_count = 0;
    alarm_id_t m_debounce_timer_id = 0;
};

#endif  // BUTTON_MANAGER_H_