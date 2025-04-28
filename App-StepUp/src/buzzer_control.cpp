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

static uint s_buzzer_pin;

int64_t melodyTimerCallback(alarm_id_t id, void *user_data);
void playNextNoteInMelody();

BuzzerControl::BuzzerControl(uint buzzer_pin)
{
    s_buzzer_pin = buzzer_pin;
    m_control_state = ControllerState::STATE_IDLE;
    m_init_success = false;
}

BuzzerControl::~BuzzerControl()
{
    deinit();
}

bool BuzzerControl::init()
{
    m_pwm_slice_num = Utils::configurePWMPin(s_buzzer_pin);

    // TODO: Ensure buzzer resets output DC signal to 0V once complete as a
    // permanent DC bias on the buzzer can damage the piezo hardware. If this
    // needs more than just setting pwm_set_enabled() to false then we should
    // consider a separate method to handle this.
    enableBuzzer(false);

    m_control_state = ControllerState::STATE_READY;

    m_init_success = true;
    return m_init_success;
}

void BuzzerControl::deinit()
{
    m_init_success = false;
}

void BuzzerControl::setBuzzerFunction(
    enum ControllerNotification controller_notification)
{
    if (m_control_state == ControllerState::STATE_READY)
    {
        m_control_state = ControllerState::STATE_BUSY;

        switch (controller_notification)
        {
            case ControllerNotification::NOTIFY_BOOT:
            {
                // s_active_melody = &melody_sweep_up;
                s_active_melody = &melody_off;
                break;
            }
            case ControllerNotification::NOTIFY_INFO:
            {
                s_active_melody = &melody_short_double_beep;
                break;
            }
            case ControllerNotification::NOTIFY_WARN:
            {
                s_active_melody = &melody_short_quadruple_beep;
                break;
            }
            case ControllerNotification::NOTIFY_ERROR:
            {
                s_active_melody = &melody_long_quadruple_beep;
                break;
            }
            case ControllerNotification::NOTIFY_POWER_DOWN:
            {
                s_active_melody = &melody_sweep_down;
                break;
            }
            default:
            {
                s_active_melody = nullptr;
                enableBuzzer(false);
                m_control_state = ControllerState::STATE_READY;
                break;
            }
        }

        if (m_control_state == ControllerState::STATE_BUSY)
        {
            // Enable the buzzer
            enableBuzzer(true);

            playNextNoteInMelody();
        }
    }
}

void BuzzerControl::enableBuzzer(bool enable)
{
    pwm_set_enabled(m_pwm_slice_num, enable);
}

enum ControllerState BuzzerControl::processJob(uint32_t tick_count)
{
    if (m_control_state == ControllerState::STATE_BUSY &&
        s_note_index_in_melody == MELODY_MAX_NOTE_COUNT)
    {
        enableBuzzer(false);
        s_note_index_in_melody = 0;
        s_active_melody = nullptr;
        // Melody was playing but has now completed, update state to represent
        // this
        m_control_state = ControllerState::STATE_READY;
    }
    return m_control_state;
}

void playNextNoteInMelody()
{
    if (s_note_index_in_melody < MELODY_MAX_NOTE_COUNT &&
        s_active_melody != nullptr)
    {
        Utils::setPWMFrequency(s_buzzer_pin,
                               s_active_melody->note[s_note_index_in_melody]);

        // Schedule next timeout/note play duration
        // TODO: Experiment with the fire_if_past flag if experiencing
        // issues... May need to add some error handling here if timer
        // cannot be created
        alarm_id_t alarm_id =
            add_alarm_in_ms(s_active_melody->duration[s_note_index_in_melody],
                            melodyTimerCallback,
                            NULL,    // user_data
                            false);  // fire_if_past

        // Only play the next note if we have valid alarm id's to use.
        // The alarm ID will be invalid (i.e. <= 0) if the timer duration is
        // too short (e.g. 0ms), as is the case when a note duration of 0ms
        // is detected.
        if (alarm_id > 0)
        {
            s_note_index_in_melody++;
        }
        else
        {
            // Terminate the melody early
            s_note_index_in_melody = MELODY_MAX_NOTE_COUNT;
        }
    }
}
int64_t melodyTimerCallback(alarm_id_t id, void *user_data)
{
    // TODO: Handle the alarm_id_t & user_data params
    playNextNoteInMelody();
    return 0;
}