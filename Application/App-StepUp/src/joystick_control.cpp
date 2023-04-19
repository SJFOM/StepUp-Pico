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
        ;
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
    ;
}
