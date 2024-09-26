/**
 * @file joystick_control.cpp
 * @author Sam (@SJFOM)
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
static volatile uint32_t s_time_of_last_button_press;
// Millisecond delay between valid button press events
static const uint32_t s_pin_debounce_delay_time_ms = 50U;

static void joystick_button_callback();

static void enableJoystickButtonInterrupt(bool enable_interrupt)
{
    gpio_set_irq_enabled(JOYSTICK_BUTTON_PIN,
                         GPIO_IRQ_EDGE_FALL,
                         enable_interrupt);  // monitor pin 1 connected to pin 0
}

/*********************************/
/* Joystick button control - END */
/*********************************/

/*************************************/
/* Joystick ADC read control - START */
/*************************************/
// Millisecond delay between valid joystick ADC read events
static const uint32_t s_adc_settling_time_between_reads_in_ms = 50U;

/***********************************/
/* Joystick ADC read control - END */
/***********************************/

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
        // Set up joystick button as interrupt: HIGH -> LOW transition
        gpio_set_input_enabled(JOYSTICK_BUTTON_PIN, true);
        gpio_pull_up(JOYSTICK_BUTTON_PIN);

        if (!Utils::isADCInitialised())
        {
            adc_init();
        }

        adc_gpio_init(JOYSTICK_ADC_PIN_X);
        adc_gpio_init(JOYSTICK_ADC_PIN_Y);
        m_joystick.position.x_offset =
            Utils::getValidADCResultRaw(JOYSTICK_ADC_CHANNEL_X);
        m_joystick.position.y_offset =
            Utils::getValidADCResultRaw(JOYSTICK_ADC_CHANNEL_Y);

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

        if ((m_joystick.position.x_offset >
                 JOYSTICK_ADC_LOWER_HOME_THRESHOLD_RAW &&
             m_joystick.position.x_offset <
                 JOYSTICK_ADC_UPPER_HOME_THRESHOLD_RAW) &&
            (m_joystick.position.y_offset >
                 JOYSTICK_ADC_LOWER_HOME_THRESHOLD_RAW &&
             m_joystick.position.y_offset <
                 JOYSTICK_ADC_UPPER_HOME_THRESHOLD_RAW))
        {
            // We assume that the joystick is roughly centered and so its
            // current position must be its "home" position
            m_joystick.position.x = 0;
            m_joystick.position.y = 0;
            m_init_success = true;
        }
        else
        {
            Utils::log_warn(
                (string) "ADC lower bound:" +
                std::to_string(JOYSTICK_ADC_LOWER_HOME_THRESHOLD_RAW));
            Utils::log_warn(
                (string) "ADC upper bound:" +
                std::to_string(JOYSTICK_ADC_UPPER_HOME_THRESHOLD_RAW));
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
        enableJoystickButtonInterrupt(true);
        gpio_add_raw_irq_handler(JOYSTICK_BUTTON_PIN,
                                 &joystick_button_callback);
        irq_set_enabled(IO_IRQ_BANK0, true);
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

    if (!isFunctionalityEnabled())
    {
        return ControllerState::STATE_IDLE;
    }

    if (s_button_press_event)
    {
        s_button_press_event = false;
        m_joystick.button_is_pressed = true;
    }

    // Get latest ADC values for both X & Y positions
    getLatestJoystickPosition();

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
        // If there has been a change in state we want to return that new data
        // is available
        m_joystick.control_state = ControllerState::STATE_NEW_DATA;
    }

    return m_joystick.control_state;
}

void JoystickControl::getLatestJoystickPosition()
{
    if (to_ms_since_boot(get_absolute_time()) >
        m_next_joystick_read_deadline_in_ms)
    {
        // Reset flag
        m_next_joystick_read_deadline_in_ms = to_ms_since_boot(
            make_timeout_time_ms(s_adc_settling_time_between_reads_in_ms));

        m_joystick.position.x =
            Utils::getValidADCResultRaw(JOYSTICK_ADC_CHANNEL_X) -
            m_joystick.position.x_offset;

        m_joystick.position.y =
            Utils::getValidADCResultRaw(JOYSTICK_ADC_CHANNEL_Y) -
            m_joystick.position.y_offset;
    }
}

int64_t debounce_timer_callback(alarm_id_t id, void *user_data)
{
    if (false == gpio_get(JOYSTICK_BUTTON_PIN))
    {
        s_button_press_event = true;
    }
    enableJoystickButtonInterrupt(true);
    return 0;
}

void joystick_button_callback()
{
    if (gpio_get_irq_event_mask(JOYSTICK_BUTTON_PIN) & GPIO_IRQ_EDGE_FALL)
    {
        gpio_acknowledge_irq(JOYSTICK_BUTTON_PIN, GPIO_IRQ_EDGE_FALL);

        // Disable interrupt until debounce timer has elapsed
        enableJoystickButtonInterrupt(false);

        // Call debounce_timer_callback in s_pin_debounce_delay_time_ms
        // milli-seconds
        add_alarm_in_ms(s_pin_debounce_delay_time_ms,
                        debounce_timer_callback,
                        NULL,
                        false);
    }
}