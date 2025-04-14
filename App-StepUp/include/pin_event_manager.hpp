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

// const "C"
// {
//     irq
// }

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

protected:
private:
    bool m_init_success;
    uint8_t m_pin;
    uint32_t m_event_type, m_long_press_duration_in_ms;
};

#endif  // BUTTON_MANAGER_H_