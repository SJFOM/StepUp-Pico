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

/***********************************/
/* Joystick button control - START */
/***********************************/
// Static non-class-member callback variables
static volatile bool s_button_press_event = false;

// Button debounce control
static uint32_t s_time_of_last_button_press;
// Millisecond delay between valid button press events
static constexpr uint8_t s_delay_time_ms = 50;

static void joystick_button_callback(uint gpio, uint32_t events);
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

        Utils::log_info((string) "X - Raw value: " +
                        std::to_string(m_joystick.position.x_offset) +
                        " - voltage: " +
                        std::to_string(m_joystick.position.x_offset *
                                       ADC_TO_VOLTAGE_CONVERSION_FACTOR) +
                        " V");
        Utils::log_info((string) "Y - Raw value: " +
                        std::to_string(m_joystick.position.y_offset) +
                        " - voltage: " +
                        std::to_string(m_joystick.position.y_offset *
                                       ADC_TO_VOLTAGE_CONVERSION_FACTOR) +
                        " V");

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
            Utils::log_warn((string) "ADC lower bound:" +
                            std::to_string(ADC_LOWER_HOME_THRESHOLD_RAW));
            Utils::log_warn((string) "ADC upper bound:" +
                            std::to_string(ADC_UPPER_HOME_THRESHOLD_RAW));
            Utils::log_warn((string) "x stage offset: " +
                            std::to_string(m_joystick.position.x_offset));
            Utils::log_warn((string) "y stage offset: " +
                            std::to_string(m_joystick.position.y_offset));
        }

        // Mimick first button press event against which to compare later events
        s_time_of_last_button_press = to_ms_since_boot(get_absolute_time());
    }

    if (m_init_success)
    {
        m_joystick.control_state = ControllerState::STATE_READY;
        // TODO: Have the ADC's constantly sample using DMA to fill a buffer
        // which we can read the averaged value from when the processJob comes
        // around to do its job
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
    // Create temporary data holder before modifying local values
    JoystickData joystick_data = m_joystick;
    m_joystick.control_state = ControllerState::STATE_READY;
    // Reset y stage as we always want to catch the state of this on each
    // iteration of processJob
    m_joystick.button_is_pressed = false;
    m_joystick.state_y = JoystickState::JOYSTICK_STATE_IDLE;
    return joystick_data;
}

enum ControllerState JoystickControl::processJob(uint32_t tick_count)
{
    enum JoystickState _joystick_state_x = m_joystick.state_x;
    enum JoystickState _joystick_state_y = m_joystick.state_y;

    if (s_button_press_event)
    {
        s_button_press_event = false;
        m_joystick.button_is_pressed = true;
    }
    adc_select_input(JOYSTICK_ADC_CHANNEL_X);
    // m_joystick.position.x = adc_read() -
    // m_joystick.position.x_offset;
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
        m_joystick.state_x = JoystickState::JOYSTICK_STATE_NEG;
    }
    else if (m_joystick.position.x > JOYSTICK_THRESHOLD_UPPER)
    {
        m_joystick.state_x = JoystickState::JOYSTICK_STATE_POS;
    }
    else
    {
        m_joystick.state_x = JoystickState::JOYSTICK_STATE_IDLE;
    }

    if (m_joystick.position.y < JOYSTICK_THRESHOLD_LOWER)
    {
        m_joystick.state_y = JoystickState::JOYSTICK_STATE_NEG;
    }
    else if (m_joystick.position.y > JOYSTICK_THRESHOLD_UPPER)
    {
        m_joystick.state_y = JoystickState::JOYSTICK_STATE_POS;
    }
    else
    {
        m_joystick.state_y = JoystickState::JOYSTICK_STATE_IDLE;
    }

    if ((_joystick_state_x != m_joystick.state_x) ||
        (_joystick_state_y != m_joystick.state_y) ||
        m_joystick.button_is_pressed)
    {
        printf("NEW DATA\n");
        if (_joystick_state_y != m_joystick.state_y)
        {
            printf("Y: %d != %d\n", _joystick_state_y, m_joystick.state_y);
        }
        // If there has been a change in state we want to return that new data
        // is available
        m_joystick.control_state = ControllerState::STATE_NEW_DATA;
    }

    return m_joystick.control_state;
}

void joystick_button_callback(uint gpio, uint32_t events)
{
    uint32_t time_now = to_ms_since_boot(get_absolute_time());
    if ((time_now - s_time_of_last_button_press) > s_delay_time_ms)
    {
        // Recommend to not to change the position of this line
        s_time_of_last_button_press = to_ms_since_boot(get_absolute_time());

        // printf("GPIO %d %d\n", gpio, events);
        s_button_press_event = true;
    }
}