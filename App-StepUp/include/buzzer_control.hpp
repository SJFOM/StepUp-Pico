/**
 * @file buzzer_control.hpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 * @license   MIT
 *
 */

#ifndef BUZZER_CONTROL_H_
#define BUZZER_CONTROL_H_

// pico-sdk
#include "hardware/pwm.h"
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`

// pin includes
#include "board_definitions.h"

// Logging utilities
#include "PicoUtils.h"

// Control libraries
#include "../../../Interfaces/ControlInterface.hpp"

#define MELODY_MAX_NOTE_COUNT      (10U)
#define BUZZER_BASE_PWM_FREQ_IN_HZ (8000U)
#define BUZZER_PWM_WRAP_VALUE      (65535U)                      // 2^16 - 1
#define BUZZER_PWM_DUTY_CYCLE      (BUZZER_PWM_WRAP_VALUE / 2U)  // 50% duty cycle

enum NoteDuration
{
    // These have all been prescribed as durations in milli-seconds
    NOTE_WHOLE = 1000U,
    NOTE_HALF = 500U,
    NOTE_QUARTER = 250U,
    NOTE_EIGHT = 125U,
    NOTE_SIXTEENTH = 63U,
    NOTE_MAX_COUNT = 5U
};

enum NotePitch
{
    // The StepUp buzzer is a piezo speaker with a center frequency of 4kHz, so
    // the frequency of the note is important. The following values represent
    // the frequencies of each note in Hz.

    NOTE_OFF = 0,     // 0 Hz = OFF
    NOTE_B6 = 1976,   // 1976 Hz
    NOTE_C7 = 2093,   // 2093 Hz
    NOTE_Cs7 = 2217,  // 2217 Hz
    NOTE_D7 = 2349,   // 2349 Hz
    NOTE_Ds7 = 2489,  // 2489 Hz
    NOTE_E7 = 2637,   // 2637 Hz
    NOTE_F7 = 2794,   // 2794 Hz
    NOTE_Fs7 = 2960,  // 2960 Hz
    NOTE_G7 = 3136,   // 3136 Hz
    NOTE_Gs7 = 3322,  // 3322 Hz
    NOTE_A7 = 3520,   // 3520 Hz
    NOTE_As7 = 3729,  // 3729 Hz
    NOTE_B7 = 3951,   // 3951 Hz
    NOTE_C8 = 4186,   // 4186 Hz
    NOTE_Cs8 = 4435,  // 4435 Hz
    NOTE_D8 = 4699,   // 4699 Hz
    NOTE_Ds8 = 4978,  // 4978 Hz
    NOTE_E8 = 5274,   // 5274 Hz
    NOTE_F8 = 5588,   // 5588 Hz
    NOTE_Fs8 = 5920,  // 5920 Hz
    NOTE_G8 = 6272,   // 6272 Hz
    NOTE_Gs8 = 6645,  // 6645 Hz
    NOTE_A8 = 7040,   // 7040 Hz
    NOTE_As8 = 7459,  // 7459 Hz
    NOTE_B8 = 7902,   // 7902 Hz
};

struct Melody
{
    uint16_t note[MELODY_MAX_NOTE_COUNT];
    uint16_t duration[MELODY_MAX_NOTE_COUNT];
};

static struct Melody melody_off = {.note = {NotePitch::NOTE_OFF},
                                   .duration = {NoteDuration::NOTE_EIGHT}};

static struct Melody melody_sweep_up = {
    .note = {NotePitch::NOTE_B6, NotePitch::NOTE_D7, NotePitch::NOTE_E7},
    .duration = {NoteDuration::NOTE_EIGHT,
                 NoteDuration::NOTE_EIGHT,
                 NoteDuration::NOTE_EIGHT}};

static struct Melody melody_sweep_down = {
    .note = {NotePitch::NOTE_E7, NotePitch::NOTE_D7, NotePitch::NOTE_B6},
    .duration = {NoteDuration::NOTE_EIGHT,
                 NoteDuration::NOTE_EIGHT,
                 NoteDuration::NOTE_EIGHT}};

static struct Melody melody_short_double_beep = {
    .note = {NotePitch::NOTE_C7,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C7,
             NotePitch::NOTE_OFF},
    .duration = {NoteDuration::NOTE_SIXTEENTH,
                 NoteDuration::NOTE_SIXTEENTH,
                 NoteDuration::NOTE_SIXTEENTH,
                 NoteDuration::NOTE_SIXTEENTH}};

static struct Melody melody_short_quadruple_beep = {
    .note = {NotePitch::NOTE_C7,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C7,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C7,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C7,
             NotePitch::NOTE_OFF},
    .duration = {
        NoteDuration::NOTE_SIXTEENTH,
        NoteDuration::NOTE_SIXTEENTH,
        NoteDuration::NOTE_SIXTEENTH,
        NoteDuration::NOTE_SIXTEENTH,
        NoteDuration::NOTE_SIXTEENTH,
        NoteDuration::NOTE_SIXTEENTH,
        NoteDuration::NOTE_SIXTEENTH,
        NoteDuration::NOTE_SIXTEENTH,
    }};

static struct Melody melody_long_quadruple_beep = {
    .note = {NotePitch::NOTE_C7,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C7,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C7,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C7,
             NotePitch::NOTE_OFF},
    .duration = {
        NoteDuration::NOTE_HALF,
        NoteDuration::NOTE_SIXTEENTH,
        NoteDuration::NOTE_HALF,
        NoteDuration::NOTE_SIXTEENTH,
        NoteDuration::NOTE_HALF,
        NoteDuration::NOTE_SIXTEENTH,
        NoteDuration::NOTE_HALF,
        NoteDuration::NOTE_SIXTEENTH,
    }};

class BuzzerControl : public ControlInterface
{
public:
    BuzzerControl(uint buzzer_pin);
    ~BuzzerControl();
    bool init();
    void deinit();
    enum ControllerState processJob(uint32_t tick_count);
    void setBuzzerFunction(enum ControllerNotification controller_notification);

protected:
private:
    bool m_init_success;
    uint16_t m_pwm_slice_num;
    enum ControllerState m_control_state;
    void enableBuzzer(bool enable);
};

#endif  // BUZZER_CONTROL_H_