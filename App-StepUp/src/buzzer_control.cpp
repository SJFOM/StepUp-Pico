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

static volatile uint8_t s_note_index_in_melody = 0;
static struct Melody *s_active_melody = nullptr;

int64_t melodyTimerCallback(alarm_id_t id, void *user_data);
void playNextNoteInMelody();

BuzzerControl::BuzzerControl()
{
    m_control_state = ControllerState::STATE_IDLE;
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
    m_pwm_slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();

    // Set divider, reduces counter clock to sysclock/this value (sysclock =
    // 125MHz default), 125MHz / 15625 = 8kHz
    pwm_config_set_clkdiv(&config, 15625.f);  // should give 8kHz div clk

    // TODO: Ensure buzzer resets output DC signal to 0V once complete as a
    // permanent DC bias on the buzzer can damage the piezo hardware. If this
    // needs more than just setting pwm_set_enabled() to false then we should
    // consider a separate method to handle this.
    disableBuzzer();

    m_control_state = ControllerState::STATE_READY;

    m_init_success = true;
    return m_init_success;
}

void BuzzerControl::deinit()
{
    m_init_success = false;
}

void BuzzerControl::setBuzzerFunction(enum BuzzerFunction buzzer_function)
{
    // TODO: Potentially need to guard against setting a new buzzer melody here
    // if one is already playing. Although, we should be able to guard against
    // this with proper ControllerState management from within the main.cpp
    // thread loop

    m_control_state = ControllerState::STATE_BUSY;

    switch (buzzer_function)
    {
        case BuzzerFunction::BUZZER_BOOT:
        {
            s_active_melody = &melody_sweep_up;
            break;
        }
        case BuzzerFunction::BUZZER_INFO:
        {
            s_active_melody = &melody_short_double_beep;
            break;
        }
        case BuzzerFunction::BUZZER_WARN:
        {
            // TODO: Create appropriate melody
            s_active_melody = &melody_short_double_beep;
            break;
        }
        case BuzzerFunction::BUZZER_ERROR:
        {
            // TODO: Create appropriate melody
            s_active_melody = &melody_short_double_beep;
            break;
        }
        case BuzzerFunction::BUZZER_OFF:
        default:
        {
            s_active_melody = nullptr;
            // TODO: If we need to power down buzzer, do it here
            m_control_state = ControllerState::STATE_READY;
            break;
        }
    }

    if (m_control_state == ControllerState::STATE_BUSY)
    {
        pwm_set_enabled(m_pwm_slice_num, true);

        playNextNoteInMelody();
    }
}

void BuzzerControl::disableBuzzer()
{
    pwm_set_enabled(m_pwm_slice_num, false);
    m_buzzer_function = BuzzerFunction::BUZZER_OFF;
}

enum ControllerState BuzzerControl::processJob(uint32_t tick_count)
{
    if (m_control_state == ControllerState::STATE_BUSY &&
        s_note_index_in_melody == MELODY_MAX_NOTE_COUNT)
    {
        disableBuzzer();
        s_note_index_in_melody = 0;
        // Melody was playing but has now completed, update state to represent
        // this
        m_control_state = ControllerState::STATE_READY;
    }
    return m_control_state;
}

void playNextNoteInMelody()
{
    if (s_note_index_in_melody < MELODY_MAX_NOTE_COUNT)
    {
        // Update with new tone
        pwm_set_gpio_level(BUZZER_PIN,
                           s_active_melody->note[s_note_index_in_melody]);

        // Schedule next timeout/note play duration
        // TODO: Experiment with the fire_if_past flag if experiencing issues...
        // May need to add some error handling here if timer cannot be created
        add_alarm_in_ms(s_active_melody->duration[s_note_index_in_melody],
                        melodyTimerCallback,
                        NULL,
                        false);

        s_note_index_in_melody++;
    }
}

int64_t melodyTimerCallback(alarm_id_t id, void *user_data)
{
    // TODO: Handle the alarm_id_t & user_data params
    playNextNoteInMelody();
    return 0;
}