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
#include "pins_definitions.h"

// Logging utilities
#include "utils.h"

// Control libraries
#include "../../../Interfaces/ControlInterface.hpp"

#define MELODY_MAX_NOTE_COUNT (10U)

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
    // These have all been prescribed assuming 8Khz = 65535. The range of notes
    // are focused around a centre frequency of 4kHz given the buzzer used on
    // the StepUp hardware resonates best at this frequency.
    NOTE_OFF = 0,      // OHz = OFF
    NOTE_B6 = 16183,   // 1976 Hz
    NOTE_C7 = 17146,   // 2093 Hz
    NOTE_Cs7 = 18165,  // 2217 Hz
    NOTE_D7 = 19245,   // 2349 Hz
    NOTE_Ds7 = 20390,  // 2489 Hz
    NOTE_E7 = 21602,   // 2637 Hz
    NOTE_F7 = 22887,   // 2794 Hz
    NOTE_Fs7 = 24248,  // 2960 Hz
    NOTE_G7 = 25689,   // 3136 Hz
    NOTE_Gs7 = 27217,  // 3322 Hz
    NOTE_A7 = 28835,   // 3520 Hz
    NOTE_As7 = 30550,  // 3729 Hz
    NOTE_B7 = 32367,   // 3951 Hz
    NOTE_C8 = 34291,   // 4186 Hz
    NOTE_Cs8 = 36330,  // 4435 Hz
    NOTE_D8 = 38491,   // 4699 Hz
    NOTE_Ds8 = 40779,  // 4978 Hz
    NOTE_E8 = 43204,   // 5274 Hz
    NOTE_F8 = 45773,   // 5588 Hz
    NOTE_Fs8 = 48495,  // 5920 Hz
    NOTE_G8 = 51379,   // 6272 Hz
    NOTE_Gs8 = 54434,  // 6645 Hz
    NOTE_A8 = 57671,   // 7040 Hz
    NOTE_As8 = 61100,  // 7459 Hz
    NOTE_B8 = 64733,   // 7902 Hz
};

struct Melody
{
    uint16_t note[MELODY_MAX_NOTE_COUNT];
    uint16_t duration[MELODY_MAX_NOTE_COUNT];
};

static struct Melody melody_sweep_up = {
    .note = {NotePitch::NOTE_E7, NotePitch::NOTE_B7, NotePitch::NOTE_F8},
    .duration = {NoteDuration::NOTE_EIGHT,
                 NoteDuration::NOTE_EIGHT,
                 NoteDuration::NOTE_EIGHT}};

static struct Melody melody_short_double_beep = {
    .note = {NotePitch::NOTE_C8,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C8,
             NotePitch::NOTE_OFF},
    .duration = {NoteDuration::NOTE_SIXTEENTH,
                 NoteDuration::NOTE_SIXTEENTH,
                 NoteDuration::NOTE_SIXTEENTH,
                 NoteDuration::NOTE_SIXTEENTH}};

static struct Melody melody_short_quadruple_beep = {
    .note = {NotePitch::NOTE_C8,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C8,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C8,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C8,
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
    .note = {NotePitch::NOTE_C8,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C8,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C8,
             NotePitch::NOTE_OFF,
             NotePitch::NOTE_C8,
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
    BuzzerControl();
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
    void disableBuzzer();
};

#endif  // BUZZER_CONTROL_H_