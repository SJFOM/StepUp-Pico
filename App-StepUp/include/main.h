/**
 * @file main.h
 * @author Sam (@SJFOM)
 * @brief Main header file for App-StepUp
 * @version 0.1
 * @date 2025-07-23
 *
 * @copyright Copyright (c) 2025
 * @license   MIT
 *
 * Adapted from:
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.4.1
 * @license   MIT
 *
 */
#ifndef MAIN_H
#define MAIN_H

// FreeRTOS
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <timers.h>
// CXX
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
// Pico SDK
#include "hardware/watchdog.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`
// App
#include <PicoUtils.h>
#include <Utils.h>
#include "board_definitions.h"
// Control includes
#include "buzzer_control.hpp"
#include "joystick_control.hpp"
#include "led_control.hpp"
#include "power_control.hpp"
#include "tmc_control.hpp"
#include "voltage_monitoring.hpp"

// Global defines
constexpr uint32_t CX_WATCHDOG_TIMEOUT_MS = 5000U;  // Max allowed is 8.3s
constexpr uint32_t CX_WATCHDOG_CALLBACK_MS = 4000U;

#ifdef __cplusplus
extern "C"
{
#endif
    // TODO: Does it make best sense to have this struct here?
    struct MotorControlData
    {
        int32_t velocity_delta;
        int8_t direction;
        bool button_press;
    };

    /**
     * PROTOTYPES
     */
    void setup();
    void setup_watchdog();
    void setup_power_control();
    void setup_led();
    void setup_tmc2300();
    void setup_boost_converter();
    void setup_joystick();
    void setup_buzzer();
    void setup_voltage_monitoring();

    void watchdog_timer_callback(TimerHandle_t xTimer);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // MAIN_H
