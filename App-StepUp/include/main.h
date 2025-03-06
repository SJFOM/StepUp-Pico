/**
 * @file main.h
 * @author Sam (@SJFOM)
 * @brief Main header file for App-StepUp
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
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
#include "pico/binary_info.h"
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`
// App
#include "board_definitions.h"
#include "utils.h"
// Control includes
#include "buzzer_control.hpp"
#include "joystick_control.hpp"
#include "led_control.hpp"
#include "tmc_control.hpp"

// Global defines
#define VELOCITY_DELTA_VALUE (500U)

#ifdef __cplusplus
extern "C"
{
#endif
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
    void setup_power_control();
    void setup_vbat_monitoring();
    void setup_vusb_monitoring();
    void setup_led();
    void setup_tmc2300();
    void setup_boost_converter();
    void setup_joystick();
    void setup_buzzer();

    void led_on();
    void led_off();
    void led_set(bool state = true);

    void led_task_pico(void *unused_arg);
    void led_task_gpio(void *unused_arg);
    void log_info(const char *msg);
    void log_device_info(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // MAIN_H
