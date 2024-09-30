/**
 * @file LEDControl.hpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2024-09-29
 *
 * @copyright Copyright (c) 2024
 * @license   MIT
 *
 */

#ifndef LED_CONTROL_H_
#define LED_CONTROL_H_

// pico-sdk
#include "hardware/pwm.h"
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`

// pin includes
#include "pins_definitions.h"

// Logging utilities
#include "utils.h"

// Control libraries
#include "../../../Interfaces/ControlInterface.hpp"

class LEDControl : public ControlInterface
{
public:
    LEDControl();
    ~LEDControl();
    bool init();
    void deinit();
    enum ControllerState processJob(uint32_t tick_count);
    void setLEDFunction(enum ControllerNotification controller_notification);

protected:
private:
    bool m_init_success;
    uint16_t m_pwm_slice_num;
    enum ControllerState m_control_state;
    void disableLED();
};

#endif  // LED_CONTROL_H_