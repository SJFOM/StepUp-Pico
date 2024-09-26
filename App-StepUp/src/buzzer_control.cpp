/**
 * @file buzzer_control.cpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 * @license   MIT
 *
 */

#include "../include/buzzer_control.hpp"

BuzzerControl::BuzzerControl()
{
    m_init_success = false;
}

BuzzerControl::~BuzzerControl()
{
    deinit();
}

bool BuzzerControl::init()
{
    // Configure the buzzer pin and direction
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();

    // Set divider, reduces counter clock to sysclock/this value (sysclock =
    // 125MHz default), 125MHz / 15625 = 8kHz
    pwm_config_set_clkdiv(&config, 15625.f);  // should give 8kHz div clk

    setBuzzerFrequency(4000);

    // Set the PWM running
    pwm_set_enabled(slice_num, true);
    sleep_ms(100);
    pwm_set_enabled(slice_num, false);
    sleep_ms(100);
    pwm_set_enabled(slice_num, true);
    sleep_ms(100);
    pwm_set_enabled(slice_num, false);

    m_init_success = true;
    return m_init_success;
}

void BuzzerControl::deinit()
{
    m_init_success = false;
}

enum ControllerState BuzzerControl::processJob(uint32_t tick_count)
{
    return ControllerState::STATE_READY;
}

void BuzzerControl::setBuzzerFrequency(uint16_t frequency_in_hz)
{
    // 4kHz = 8000/2 = 65535/2 ~= 32768 = 2^15

    // Base clock has been divided to an 8kHz clock which is 16 bits wide
    // (65535) The means we must scale the 8kHz bandwidth over 65535 chunks,
    // each with a frequency band of ~0.12 Hz (our resolution)

    // TODO: Speed up this operation, don't need accuracy here - can pre-compute
    // some of these values if needed
    uint16_t pwm_level = frequency_in_hz * 65535 / 8000;
    pwm_set_gpio_level(BUZZER_PIN, pwm_level);
}