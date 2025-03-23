/**
 * @file power_control.cpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2025-03-23
 *
 * @copyright Copyright (c) 2025
 * @license   MIT
 *
 */

#include "../include/power_control.hpp"

/********************************/
/* Power button control - START */
/********************************/
// TODO: Consider amalgamating debounce functionality into single
// library/utility Static non-class-member callback variables
static volatile bool s_button_press_event = false;

// Button debounce control
static volatile uint32_t s_time_of_last_button_press;
// Millisecond delay between valid button press events
static const uint32_t s_pin_debounce_delay_time_ms = 50U;

// USB plug in/out callback method
static void usb_detect_callback();

static void enableJoystickButtonInterrupt(bool enable_interrupt)
{
    gpio_set_irq_enabled(JOYSTICK_BUTTON_PIN,
                         GPIO_IRQ_EDGE_FALL,
                         enable_interrupt);  // monitor pin 1 connected to pin 0
}

/******************************/
/* Power button control - END */
/******************************/

PowerControl::PowerControl() : m_init_success(false) {};

PowerControl::~PowerControl()
{
    deinit();
}

bool PowerControl::init()
{
    // Avoid multiple calls to this method performing the same configuration
    // steps
    if (false == m_init_success)
    {
        // Set up Power button as interrupt: HIGH -> LOW transition
        gpio_set_input_enabled(JOYSTICK_BUTTON_PIN, true);
        gpio_pull_up(JOYSTICK_BUTTON_PIN);

        if (!Utils::isADCInitialised())
        {
            adc_init();
        }
    }

    return m_init_success;
}

void PowerControl::deinit()
{
    // Not sure what this would even do...
    ;
}

enum ControllerState PowerControl::processJob(uint32_t tick_count)
{
    return ControllerState::STATE_IDLE;
}