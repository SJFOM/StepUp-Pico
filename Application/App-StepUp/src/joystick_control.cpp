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

static volatile bool button_press_event = false;

void joystick_button_callback(uint gpio, uint32_t events);

JoystickControl::JoystickControl()
{
    m_joystick.joystick_state = JOYSTICK_STATE_IDLE;
    m_joystick.button_is_pressed = false;
    m_init_success = false;

    m_joystick.current = 5;  // Default IRUN value
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
        gpio_set_input_enabled(JOYSTICK_BUTTON_PIN, true);
        gpio_pull_up(JOYSTICK_BUTTON_PIN);
        // Set up the joystick button interrupt
        gpio_set_irq_enabled_with_callback(JOYSTICK_BUTTON_PIN,
                                           GPIO_IRQ_EDGE_FALL,
                                           true,
                                           &joystick_button_callback);

        adc_init();

        // Make sure GPIO is high-impedance, no pullups etc
        adc_gpio_init(JOYSTICK_ADC_PIN_X);
        adc_gpio_init(JOYSTICK_ADC_PIN_Y);

        adc_select_input(JOYSTICK_ADC_CHANNEL_X);
        m_joystick.position.x_offset = adc_read();

        adc_select_input(JOYSTICK_ADC_CHANNEL_Y);
        m_joystick.position.y_offset = adc_read();

        printf("X - Raw value: 0x%03x, voltage: %f V\n",
               m_joystick.position.x_offset,
               m_joystick.position.x_offset * adc_conversion_factor);
        printf("Y - Raw value: 0x%03x, voltage: %f V\n",
               m_joystick.position.y_offset,
               m_joystick.position.y_offset * adc_conversion_factor);

        if ((m_joystick.position.x_offset > ADC_LOWER_HOME_THRESHOLD_RAW &&
             m_joystick.position.x_offset < ADC_UPPER_HOME_THRESHOLD_RAW) &&
            (m_joystick.position.y_offset > ADC_LOWER_HOME_THRESHOLD_RAW &&
             m_joystick.position.y_offset < ADC_UPPER_HOME_THRESHOLD_RAW))
        {
            // We assume that the joystick is roughly centered and so its
            // current position must be its "home" position
            m_joystick.position.x = 0;
            m_joystick.position.y = 0;
            m_init_success = true;
        }
        else
        {
            printf("ADC lower bound = %d\n", ADC_LOWER_HOME_THRESHOLD_RAW);
            printf("x stage offset = %d\n", m_joystick.position.x_offset);
            printf("y stage offset = %d\n", m_joystick.position.y_offset);
        }
    }

    if (m_init_success)
    {
        m_joystick.control_state = ControllerState::STATE_READY;
        // adc_set_round_robin(JOYSTICK_ADC_ROUND_ROBIN_MASK);
        // Set channel 0 to be first
        // adc_select_input(JOYSTICK_ADC_CHANNEL_X);
    }

    return m_init_success;
}

void JoystickControl::deinit()
{
    // Reset the Joystick to its default values and state
    m_joystick.position.x = 0;
    m_joystick.position.x_offset = 0;
    m_joystick.position.y = 0;
    m_joystick.position.y_offset = 0;

    // De-initialise the joystick pins
    gpio_disable_pulls(JOYSTICK_BUTTON_PIN);

    m_joystick.control_state = ControllerState::STATE_IDLE;

    m_init_success = false;
}

enum JoystickState JoystickControl::getJoystickState()
{
    m_joystick.control_state = ControllerState::STATE_READY;
    return m_joystick.joystick_state;
}

struct JoystickData JoystickControl::getJoystickData()
{
    m_joystick.control_state = ControllerState::STATE_READY;
    return m_joystick;
}

enum ControllerState JoystickControl::processJob(uint32_t tick_count)
{
    enum JoystickState _joystick_state = m_joystick.joystick_state;
    uint8_t _current = m_joystick.current;
    if (button_press_event)
    {
        // TODO: Implement some de-bounce mechanism
        button_press_event = false;
        m_joystick.button_is_pressed = true;
    }

    adc_select_input(JOYSTICK_ADC_CHANNEL_X);
    // m_joystick.position.x = adc_read() - m_joystick.position.x_offset;
    m_joystick.position.x = adc_read() & ADC_ENOB_MASK;
    // printf("x stage before = %d\n", m_joystick.position.x);
    m_joystick.position.x -= m_joystick.position.x_offset;
    // printf("x stage after = %d\n", m_joystick.position.x);

    int16_t x = abs(m_joystick.position.x);

    adc_select_input(JOYSTICK_ADC_CHANNEL_Y);
    m_joystick.position.y =
        (adc_read() & ADC_ENOB_MASK) - m_joystick.position.y_offset;

    // printf("y stage after = %d\n", m_joystick.position.y);

    // int8_t direction = (m_joystick.position.x > 0) ? 1 : -1;

    if (x >= 100 && x < 1000)
    {
        m_joystick.joystick_state = JoystickState::JOYSTICK_STATE_LOW;
    }
    else if (x > 1000 && x < 1800)
    {
        m_joystick.joystick_state = JoystickState::JOYSTICK_STATE_MID;
    }
    else if (x > 1800)
    {
        m_joystick.joystick_state = JoystickState::JOYSTICK_STATE_HIGH;
    }
    else
    {
        m_joystick.joystick_state = JoystickState::JOYSTICK_STATE_IDLE;
    }

    if (m_joystick.position.y < -1800)
    {
        if (m_joystick.current > 0)
        {
            m_joystick.current--;
        }
    }
    else if (m_joystick.position.y > 1800)
    {
        if (m_joystick.current < 31)
        {
            m_joystick.current++;
        }
    }

    if (_joystick_state != m_joystick.joystick_state ||
        _current != m_joystick.current || m_joystick.button_is_pressed)
    {
        // If there has been a change in state we want to return that new data
        // is available
        m_joystick.control_state = ControllerState::STATE_NEW_DATA;
    }

    return m_joystick.control_state;
}

void joystick_button_callback(uint gpio, uint32_t events)
{
    button_press_event = true;
    printf("GPIO %d %d\n", gpio, events);
}