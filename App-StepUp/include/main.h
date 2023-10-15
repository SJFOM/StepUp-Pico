/**
 * Main header file for App-StepUp
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
#include "pins_definitions.h"
#include "utils.h"
// Control includes
#include "joystick_control.hpp"
#include "tmc_control.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * PROTOTYPES
     */
    void setup();
    void setup_led();
    void setup_tmc2300();
    void setup_joystick();

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
