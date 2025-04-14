/**
 * @file pin_event_manager.cpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2025-03-23
 *
 * @copyright Copyright (c) 2025
 * @license   MIT
 *
 */

#include "../include/pin_event_manager.hpp"

PinEventManager::PinEventManager(uint8_t pin,
                                 uint32_t event_type,
                                 uint32_t long_press_duration_in_ms) {};

PinEventManager::~PinEventManager()
{
    deinit();
}

bool PinEventManager::init()
{
    return true;
    // return m_init_success;
}

void PinEventManager::deinit()
{
    // m_init_success = false;
}

// int64_t debounce_timer_callback(alarm_id_t id, void *user_data)
// {
// if (false == gpio_get(m_pin))
// {
//     s_button_press_event = true;
// }
// enableJoystickButtonInterrupt(true);
// return 0;
// }

// void joystick_button_callback()
// {
// if (gpio_get_irq_event_mask(m_pin) & GPIO_IRQ_EDGE_FALL)
// {
//     gpio_acknowledge_irq(m_pin, GPIO_IRQ_EDGE_FALL);

//     // Disable interrupt until debounce timer has elapsed
//     enableJoystickButtonInterrupt(false);

//     // Call debounce_timer_callback in
//     s_pin_debounce_default_delay_time_ms
//     // milli-seconds
//     add_alarm_in_ms(s_pin_debounce_default_delay_time_ms,
//                     debounce_timer_callback,
//                     NULL,
//                     false);
// }
// }

// void usb_detect_callback()
// {
//     if (gpio_get_irq_event_mask(VUSB_MONITOR_PIN) & (GPIO_IRQ_EDGE_RISE))
//     {
//         gpio_acknowledge_irq(VUSB_MONITOR_PIN, (GPIO_IRQ_EDGE_RISE));
//         s_usb_is_inserted = true;
//     }
//     if (gpio_get_irq_event_mask(VUSB_MONITOR_PIN) & (GPIO_IRQ_EDGE_FALL))
//     {
//         gpio_acknowledge_irq(VUSB_MONITOR_PIN, (GPIO_IRQ_EDGE_FALL));
//         s_usb_is_inserted = false;
//     }
// }