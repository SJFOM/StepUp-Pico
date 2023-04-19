/**
 * @file joystick_control.cpp
 * @author SJFOM (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-04-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../include/joystick_control.hpp"


// 12-bit conversion, assume max value == ADC_VREF == 3.3 V
constexpr float adc_conversion_factor = 3.3f / (1 << 12);

JoystickControl::JoystickControl()
{
    m_init_success = false;
}
JoystickControl::~JoystickControl()
{
    deinit();
}

bool JoystickControl::init()
{
    // Avoid multiple calls to this method performing the same configuration
    // steps
    if (false == m_init_success)
    {
        adc_init();
        
        // Make sure GPIO is high-impedance, no pullups etc
        adc_gpio_init(ADC_PIN_JOYSTICK_X);
        adc_gpio_init(ADC_PIN_JOYSTICK_Y); 

        adc_select_input(0);
        joystick_pos.x_offset = adc_read();
        
        adc_select_input(1);
        joystick_pos.y_offset = adc_read();

        printf("X - Raw value: 0x%03x, voltage: %f V\n", joystick_pos.x_offset, joystick_pos.x_offset * adc_conversion_factor);
        printf("Y - Raw value: 0x%03x, voltage: %f V\n", joystick_pos.y_offset, joystick_pos.y_offset * adc_conversion_factor);

        if((joystick_pos.x_offset > ADC_LOWER_HOME_THRESHOLD_RAW && joystick_pos.x_offset < ADC_UPPER_HOME_THRESHOLD_RAW) 
            && (joystick_pos.y_offset > ADC_LOWER_HOME_THRESHOLD_RAW && joystick_pos.y_offset < ADC_UPPER_HOME_THRESHOLD_RAW))
        {
            // We assume that the joystick is roughly centered and so its current position must be its "home" position
            joystick_pos.x = 0;
            joystick_pos.y = 0;
            m_init_success = true;
        }
    }
    return m_init_success;
}

void JoystickControl::deinit()
{
    // TODO: Reset the Joystick to its default values and state
    ;

    // TODO: De-initialise the ADC peripheral
    ;

    m_init_success = false;
}


void JoystickControl::processJob(uint32_t tick_count)
{
    adc_select_input(0);
    uint16_t adc_x_raw = adc_read();
    adc_select_input(1);
    uint16_t adc_y_raw = adc_read();
}
