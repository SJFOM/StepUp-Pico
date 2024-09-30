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

LEDControl::LEDControl()
{
    m_control_state = ControllerState::STATE_IDLE;
    m_init_success = false;
}

LEDControl::~LEDControl()
{
    deinit();
}

bool LEDControl::init()
{
    // Configure the led pins and direction
    gpio_set_function(LED_PIN_BLUE, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to LED_PIN
    m_pwm_slice_num = pwm_gpio_to_slice_num(LED_PIN_BLUE);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();

    // Set divider, reduces counter clock to sysclock/this value (sysclock =
    // 125MHz default), 125MHz / 15625 = 8kHz
    pwm_config_set_clkdiv(&config, 15625.f);  // should give 8kHz div clk

    disableLED();

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
    ;
}

void LEDControl::disableLED()
{
    pwm_set_enabled(m_pwm_slice_num, false);
}

enum ControllerState LEDControl::processJob(uint32_t tick_count)
{
    return m_control_state;
}