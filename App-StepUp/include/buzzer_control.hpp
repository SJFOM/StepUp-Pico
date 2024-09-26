/**
 * @file buzzer_control.hpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 * @license   MIT
 *
 */

#ifndef BUZZER_CONTROL_H_
#define BUZZER_CONTROL_H_

// pico-sdk
#include "hardware/pwm.h"
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`

// pin includes
#include "pins_definitions.h"

// Logging utilities
#include "utils.h"

// Control libraries
#include "../../../Interfaces/ControlInterface.hpp"

class BuzzerControl : public ControlInterface
{
public:
    BuzzerControl();
    ~BuzzerControl();
    bool init();
    void deinit();
    enum ControllerState processJob(uint32_t tick_count);

protected:
private:
    bool m_init_success;
    void setBuzzerFrequency(uint16_t frequency_in_hz);
};

#endif  // JOYSTICK_CONTROL_H_