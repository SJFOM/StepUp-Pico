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
#include "board_definitions.h"

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

enum LEDDuration
{
    // These have all been prescribed as durations in milli-seconds
    LED_SLOW = 1000U,
    LED_MEDIUM = 500U,
    LED_FAST = 150U,
    LED_VERY_FAST = 80U
};

// FIXME: This will keep the LED ON only by virtue of the fact that the length
// of the array is LED_MAX_TRANSITION_COUNT. Otherwise, the effect will be empty
// (0) and turn the LED off
static struct LEDEffect effect_fade_to_on = {
    .effect = {0, 4000, 8000, 0, 4000, 8000, 0, 4000, 8000, 4000},
    .duration = {LEDDuration::LED_FAST,
                 LEDDuration::LED_FAST,
                 LEDDuration::LED_FAST,
                 LEDDuration::LED_FAST,
                 LEDDuration::LED_FAST,
                 LEDDuration::LED_FAST,
                 LEDDuration::LED_FAST,
                 LEDDuration::LED_FAST,
                 LEDDuration::LED_FAST,
                 LEDDuration::LED_FAST}};

static struct LEDEffect effect_blink = {.effect = {LED_BASE_PWM_FREQ_IN_HZ,
                                                   0,
                                                   LED_BASE_PWM_FREQ_IN_HZ,
                                                   0,
                                                   LED_BASE_PWM_FREQ_IN_HZ,
                                                   0},
                                        .duration = {LEDDuration::LED_FAST,
                                                     LEDDuration::LED_FAST,
                                                     LEDDuration::LED_FAST,
                                                     LEDDuration::LED_FAST,
                                                     LEDDuration::LED_FAST,
                                                     LEDDuration::LED_FAST}};

static struct LEDEffect effect_rapid_blink = {
    .effect = {LED_BASE_PWM_FREQ_IN_HZ,
               0,
               LED_BASE_PWM_FREQ_IN_HZ,
               0,
               LED_BASE_PWM_FREQ_IN_HZ,
               0},
    .duration = {LEDDuration::LED_VERY_FAST,
                 LEDDuration::LED_VERY_FAST,
                 LEDDuration::LED_VERY_FAST,
                 LEDDuration::LED_VERY_FAST,
                 LEDDuration::LED_VERY_FAST,
                 LEDDuration::LED_VERY_FAST}};

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