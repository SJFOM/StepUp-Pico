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

#define LED_MAX_TRANSITION_COUNT (10U)
#define LED_BASE_PWM_FREQ_IN_HZ  (8000U)

struct LEDEffect
{
    uint16_t effect[LED_MAX_TRANSITION_COUNT];
    uint16_t duration[LED_MAX_TRANSITION_COUNT];
};

struct RGBPinHandlers
{
    uint led_pin_red, led_pin_green, led_pin_blue;
    uint16_t led_red_pwm_slice, led_green_pwm_slice, led_blue_pwm_slice;
    uint led_pin_in_use;
    uint16_t led_pwm_slice_in_use;
};

static struct LEDEffect effect_fade_to_on = {
    .effect = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000},
    .duration = {200, 200, 200, 200, 200, 200, 200, 200}};

static struct LEDEffect effect_blink = {
    .effect = {LED_BASE_PWM_FREQ_IN_HZ,
               0,
               LED_BASE_PWM_FREQ_IN_HZ,
               0,
               LED_BASE_PWM_FREQ_IN_HZ,
               0},
    .duration = {200, 200, 200, 200, 200, 200}};

static struct LEDEffect effect_rapid_blink = {
    .effect = {LED_BASE_PWM_FREQ_IN_HZ,
               0,
               LED_BASE_PWM_FREQ_IN_HZ,
               0,
               LED_BASE_PWM_FREQ_IN_HZ,
               0},
    .duration = {100, 100, 100, 100, 100, 100}};

class LEDControl : public ControlInterface
{
public:
    LEDControl(uint led_red_pin, uint led_green_pin, uint led_blue_pin);
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