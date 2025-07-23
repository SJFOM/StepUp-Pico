/**
 * @file LEDControl.hpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2025-07-23
 *
 * @copyright Copyright (c) 2025
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
#include "PicoUtils.h"

// Control libraries
#include "ControlInterface.hpp"

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
    LED_SLOW = 500U,
    LED_MEDIUM = 300U,
    LED_FAST = 100U,
    LED_VERY_FAST = 50U
};

// NOTE: This will keep the LED ON only by virtue of the fact that the length
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

enum LEDColourNames
{
    LED_COLOUR_RED = 0U,
    LED_COLOUR_GREEN = 1U,
    LED_COLOUR_BLUE = 2U,
    LED_COLOUR_YELLOW = 3U,
    LED_COLOUR_ORANGE = 4U,
    LED_COLOUR_CYAN = 5U,
    LED_COLOUR_MAGENTA = 6U,
    LED_COLOUR_WHITE = 7U,
    LED_COLOUR_OFF = 8U,
    LED_COLOUR_MAX_COUNT
};
struct LEDColourData
{
    enum LEDColourNames led_colour_name;  // enum containing supported colours
    uint16_t red, green, blue;            // Colour intensities (0-65535)
};

// Predefined LED colors
static const LEDColourData LEDColours[] = {
    {LEDColourNames::LED_COLOUR_RED, 65535, 0, 0},
    {LEDColourNames::LED_COLOUR_GREEN, 0, 65535, 0},
    {LEDColourNames::LED_COLOUR_BLUE, 0, 0, 65535},
    {LEDColourNames::LED_COLOUR_YELLOW, 65535, 65535, 0},
    {LEDColourNames::LED_COLOUR_ORANGE, 65535, 20000, 0},
    {LEDColourNames::LED_COLOUR_CYAN, 0, 65535, 65535},
    {LEDColourNames::LED_COLOUR_MAGENTA, 65535, 0, 65535},
    {LEDColourNames::LED_COLOUR_WHITE, 65535, 65535, 65535},
    {LEDColourNames::LED_COLOUR_OFF, 0, 0, 0}  // Turn off the LED
};

class LEDControl : public ControlInterface
{
public:
    LEDControl(uint led_red_pin, uint led_green_pin, uint led_blue_pin);
    ~LEDControl();
    bool init();
    void deinit();
    enum ControllerState processJob(uint32_t tick_count);
    void setLEDFunction(enum ControllerNotification controller_notification);
    void setLEDColour(enum LEDColourNames led_colour);

protected:
private:
    bool m_init_success;
    uint16_t m_pwm_slice_num;
    enum ControllerState m_control_state;
    enum LEDColourNames m_active_colour_name;
    void enableLED(bool enable);
};

#endif  // LED_CONTROL_H_