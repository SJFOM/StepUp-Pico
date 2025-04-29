/**
 * @file led_control.cpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2024-09-29
 *
 * @copyright Copyright (c) 2024
 * @license   MIT
 *
 */

#include "../include/led_control.hpp"

static volatile uint8_t s_effect_index_in_transition = 0;
static struct LEDEffect *s_active_effect = nullptr;

static RGBPinHandlers s_rgb_led;

int64_t ledTimerCallback(alarm_id_t id, void *user_data);
void queueNextLEDEffect();

LEDControl::LEDControl(uint led_red_pin, uint led_green_pin, uint led_blue_pin)
{
    s_rgb_led.led_pin_red = led_red_pin;
    s_rgb_led.led_pin_green = led_green_pin;
    s_rgb_led.led_pin_blue = led_blue_pin;
    m_control_state = ControllerState::STATE_IDLE;
    m_init_success = false;
}

LEDControl::~LEDControl()
{
    deinit();
}

bool LEDControl::init()
{
    s_rgb_led.led_red_pwm_slice = Utils::configurePWMPin(s_rgb_led.led_pin_red);
    Utils::setPWMFrequency(s_rgb_led.led_pin_red, LED_BASE_PWM_FREQ_IN_HZ);
    s_rgb_led.led_green_pwm_slice =
        Utils::configurePWMPin(s_rgb_led.led_pin_green);
    Utils::setPWMFrequency(s_rgb_led.led_pin_green, LED_BASE_PWM_FREQ_IN_HZ);
    s_rgb_led.led_blue_pwm_slice =
        Utils::configurePWMPin(s_rgb_led.led_pin_blue);
    Utils::setPWMFrequency(s_rgb_led.led_pin_blue, LED_BASE_PWM_FREQ_IN_HZ);

    enableLED(false);

    m_control_state = ControllerState::STATE_READY;

    m_init_success = true;
    return m_init_success;
}

void LEDControl::deinit()
{
    m_init_success = false;
}

void LEDControl::setLEDFunction(
    enum ControllerNotification controller_notification)

{
    if (m_control_state == ControllerState::STATE_READY)
    {
        m_control_state = ControllerState::STATE_BUSY;

        switch (controller_notification)
        {
            case ControllerNotification::NOTIFY_BOOT:
            {
                s_active_effect = &effect_fade_to_on;
                s_rgb_led.led_pin_in_use = s_rgb_led.led_pin_green;
                s_rgb_led.led_pwm_slice_in_use = s_rgb_led.led_green_pwm_slice;
                break;
            }
            case ControllerNotification::NOTIFY_INFO:
            {
                s_active_effect = &effect_blink;
                s_rgb_led.led_pin_in_use = s_rgb_led.led_pin_blue;
                s_rgb_led.led_pwm_slice_in_use = s_rgb_led.led_blue_pwm_slice;
                break;
            }
            case ControllerNotification::NOTIFY_WARN:
            {
                s_active_effect = &effect_blink;
                s_rgb_led.led_pin_in_use = s_rgb_led.led_pin_red;
                s_rgb_led.led_pwm_slice_in_use = s_rgb_led.led_red_pwm_slice;
                break;
            }
            case ControllerNotification::NOTIFY_ERROR:
            {
                s_active_effect = &effect_rapid_blink;
                s_rgb_led.led_pin_in_use = s_rgb_led.led_pin_red;
                s_rgb_led.led_pwm_slice_in_use = s_rgb_led.led_red_pwm_slice;
                break;
            }
            case ControllerNotification::NOTIFY_POWER_DOWN:
            {
                s_active_effect = &effect_blink;
                s_rgb_led.led_pin_in_use = s_rgb_led.led_pin_red;
                s_rgb_led.led_pwm_slice_in_use = s_rgb_led.led_red_pwm_slice;
                break;
            }
            default:
            {
                s_active_effect = nullptr;
                enableLED(false);
                m_control_state = ControllerState::STATE_READY;
                break;
            }
        }

        if (m_control_state == ControllerState::STATE_BUSY)
        {
            // Firstly, power off all LED's to ensure we only show the newly
            // active LED
            setLEDColour(LEDColourNames::LED_COLOUR_OFF);

            // Enable the led
            pwm_set_enabled(s_rgb_led.led_pwm_slice_in_use, true);

            // Reset the effect index to start a new pattern
            s_effect_index_in_transition = 0;

            queueNextLEDEffect();
        }
    }
}

void LEDControl::setLEDColour(enum LEDColourNames led_colour)
{
    for (const auto &colour : LEDColours)
    {
        if (colour.led_colour_name == led_colour)
        {
            pwm_set_gpio_level(s_rgb_led.led_pin_red, colour.red);
            pwm_set_gpio_level(s_rgb_led.led_pin_green, colour.green);
            pwm_set_gpio_level(s_rgb_led.led_pin_blue, colour.blue);

            m_active_colour_name = led_colour;

            enableLED(true);
            break;
        }
    }
}

void LEDControl::enableLED(bool enable)
{
    pwm_set_enabled(s_rgb_led.led_red_pwm_slice, enable);
    pwm_set_enabled(s_rgb_led.led_green_pwm_slice, enable);
    pwm_set_enabled(s_rgb_led.led_blue_pwm_slice, enable);
}

enum ControllerState LEDControl::processJob(uint32_t tick_count)
{
    if (m_control_state == ControllerState::STATE_BUSY &&
        s_effect_index_in_transition == LED_MAX_TRANSITION_COUNT)
    {
        s_effect_index_in_transition = 0;
        s_active_effect = nullptr;
        // LED transition was ongoing but has now completed

        // Re-instate LED colour
        setLEDColour(m_active_colour_name);

        // Update state to represent this
        m_control_state = ControllerState::STATE_READY;
    }
    return m_control_state;
}

void queueNextLEDEffect()
{
    if (s_effect_index_in_transition < LED_MAX_TRANSITION_COUNT &&
        s_active_effect != nullptr)
    {
        // Update with new led effect
        pwm_set_gpio_level(
            s_rgb_led.led_pin_in_use,
            s_active_effect->effect[s_effect_index_in_transition]);

        // Schedule next timeout/effect show duration
        // TODO: Experiment with the fire_if_past flag if experiencing issues...
        // May need to add some error handling here if timer cannot be created
        alarm_id_t alarm_id = add_alarm_in_ms(
            s_active_effect->duration[s_effect_index_in_transition],
            ledTimerCallback,
            NULL,    // user_data
            false);  // fire_if_past

        // Only play the next effect if we have valid alarm id's to use.
        // The alarm ID will be invalid (i.e. <= 0) if the timer duration is too
        // short (e.g. 0ms), as is the case when a effect duration of 0ms is
        // detected.
        if (alarm_id > 0)
        {
            s_effect_index_in_transition++;
        }
        else
        {
            // Terminate the melody early
            s_effect_index_in_transition = LED_MAX_TRANSITION_COUNT;
        }
    }
}

int64_t ledTimerCallback(alarm_id_t id, void *user_data)
{
    // TODO: Handle the alarm_id_t & user_data params
    queueNextLEDEffect();
    return 0;
}