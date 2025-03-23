/**
 * @file power_control.hpp
 * @author Sam (@SJFOM)
 * @brief Monitoring and controlling device power state, observing USB port use
 * and power button user interactions
 * @version 0.1
 * @date 2025-03-23
 *
 * @copyright Copyright (c) 2025
 * @license   MIT
 *
 */

#ifndef POWER_CONTROL_H_
#define POWER_CONTROL_H_

// pico-sdk
#include "hardware/gpio.h"
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`

// pin includes
#include "board_definitions.h"

// Logging utilities
#include "utils.h"

// Control libraries
#include "../../../Interfaces/ControlInterface.hpp"

constexpr static uint32_t c_s_power_button_off_hold_time_ms = (3000U);

enum class PowerState
{
    POWER_STATE_IDLE = 0,
    POWER_STATE_USB_INSERTED = 1U,
    POWER_STATE_BUTTON_INTERACTION = 2U,
    POWER_STATE_BUTTON_POWER_OFF = 3U,
    POWER_STATE_MAX_COUNT,
}

class PowerControl : public ControlInterface
{
public:
    PowerControl();
    ~PowerControl();
    bool init();
    void deinit();
    enum ControllerState processJob(uint32_t tick_count);

    void powerOff();

bool

    protected : private : bool m_init_success;
};

#endif  // POWER_CONTROL_H_