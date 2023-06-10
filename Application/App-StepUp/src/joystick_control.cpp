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

/***********************************/
/* Joystick button control - START */
/***********************************/
static volatile bool button_press_event = false;

// Button debounce control
uint32_t time_of_last_button_press;
// Millisecond delay between valid button press events
constexpr uint8_t delay_time_ms = 50;

void joystick_button_callback(uint gpio, uint32_t events);
/*********************************/
/* Joystick button control - END */
/*********************************/

JoystickControl::JoystickControl()
{
    m_joystick.state_x = JOYSTICK_STATE_IDLE;
    m_joystick.state_y = JOYSTICK_STATE_IDLE;
    m_joystick.button_is_pressed = false;
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

        // Mimick first button press event against which to compare later events
        time_of_last_button_press = to_ms_since_boot(get_absolute_time());
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

struct JoystickData JoystickControl::getJoystickData()
{
    m_joystick.control_state = ControllerState::STATE_READY;
    JoystickData temp_data = m_joystick;
    m_joystick.state_x = JoystickState::JOYSTICK_STATE_IDLE;
    m_joystick.state_y = JoystickState::JOYSTICK_STATE_IDLE;
    m_joystick.button_is_pressed = false;
    return temp_data;
}

enum ControllerState JoystickControl::processJob(uint32_t tick_count)
{
    enum JoystickState _joystick_state_x = m_joystick.state_x;
    enum JoystickState _joystick_state_y = m_joystick.state_y;
    if (button_press_event)
    {
        button_press_event = false;
        m_joystick.button_is_pressed = true;
    }

    adc_select_input(JOYSTICK_ADC_CHANNEL_X);
    // m_joystick.position.x = adc_read() - m_joystick.position.x_offset;
    m_joystick.position.x = adc_read() & ADC_ENOB_MASK;
    // printf("x stage before = %d\n", m_joystick.position.x);
    m_joystick.position.x -= m_joystick.position.x_offset;
    // printf("x stage after = %d\n", m_joystick.position.x);

    adc_select_input(JOYSTICK_ADC_CHANNEL_Y);
    m_joystick.position.y =
        (adc_read() & ADC_ENOB_MASK) - m_joystick.position.y_offset;
    // printf("y stage after = %d\n", m_joystick.position.y);

    if (m_joystick.position.x < JOYSTICK_THRESHOLD_LOWER)
    {
        // printf("XL\n");
        m_joystick.state_x = JoystickState::JOYSTICK_STATE_NEG;
    }
    else if (m_joystick.position.x > JOYSTICK_THRESHOLD_UPPER)
    {
        // printf("XU\n");
        m_joystick.state_x = JoystickState::JOYSTICK_STATE_POS;
    }
    else
    {
        // printf("XI\n");
        m_joystick.state_x = JoystickState::JOYSTICK_STATE_IDLE;
    }

    if (m_joystick.position.y < JOYSTICK_THRESHOLD_LOWER)
    {
        // printf("YL\n");
        m_joystick.state_y = JoystickState::JOYSTICK_STATE_NEG;
    }
    else if (m_joystick.position.y > JOYSTICK_THRESHOLD_UPPER)
    {
        // printf("YU\n");
        m_joystick.state_y = JoystickState::JOYSTICK_STATE_POS;
    }
    else
    {
        // printf("YI\n");
        m_joystick.state_y = JoystickState::JOYSTICK_STATE_IDLE;
    }

    if ((_joystick_state_x != m_joystick.state_x) ||
        (_joystick_state_y != m_joystick.state_y) ||
        m_joystick.button_is_pressed)
    {
        // If there has been a change in state we want to return that new data
        // is available
        m_joystick.control_state = ControllerState::STATE_NEW_DATA;
    }

    return m_joystick.control_state;
}

void joystick_button_callback(uint gpio, uint32_t events)
{
    uint32_t time_now = to_ms_since_boot(get_absolute_time());
    if ((time_now - time_of_last_button_press) > delay_time_ms)
    {
        // Recommend to not to change the position of this line
        time_of_last_button_press = to_ms_since_boot(get_absolute_time());

        printf("GPIO %d %d\n", gpio, events);
        button_press_event = true;
    }
}