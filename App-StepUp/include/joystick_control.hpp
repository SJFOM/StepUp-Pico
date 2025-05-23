/**
 * @file joystick_control.hpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 * @license   MIT
 *
 */

#ifndef JOYSTICK_CONTROL_H_
#define JOYSTICK_CONTROL_H_

// pico-sdk
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`

// pin includes
#include "board_definitions.h"

// Logging utilities
#include "PicoUtils.h"

// Control libraries
#include "ControlInterface.hpp"

// PinManager
#include "pin_event_manager.hpp"

#define JOYSTICK_ADC_HOME_THRESHOLD_RAW (200U)
#define JOYSTICK_ADC_LOWER_HOME_THRESHOLD_RAW \
    (ADC_MIDWAY_VALUE_RAW - JOYSTICK_ADC_HOME_THRESHOLD_RAW)
#define JOYSTICK_ADC_UPPER_HOME_THRESHOLD_RAW \
    (ADC_MIDWAY_VALUE_RAW + JOYSTICK_ADC_HOME_THRESHOLD_RAW)

#define JOYSTICK_THRESHOLD_UPPER (1000)
#define JOYSTICK_THRESHOLD_LOWER (-JOYSTICK_THRESHOLD_UPPER)

struct JoystickPosition
{
    int16_t x, y;
    uint16_t x_offset, y_offset;
};

enum JoystickState
{
    JOYSTICK_STATE_NEG = -1,
    JOYSTICK_STATE_IDLE = 0,
    JOYSTICK_STATE_POS = 1,
};

struct JoystickData
{
    JoystickPosition position;
    enum JoystickState state_x, state_y;
    enum ControllerState control_state;
    bool button_is_pressed;
};

class JoystickControl : public ControlInterface
{
public:
    JoystickControl();
    ~JoystickControl();
    bool init();
    void deinit();
    enum ControllerState processJob(uint32_t tick_count);
    struct JoystickData getJoystickData();

protected:
private:
    struct JoystickData m_joystick;
    uint32_t m_next_joystick_read_deadline_in_ms;

    PinEventManager *m_pin_event_manager = nullptr;

    void getLatestJoystickPosition();
};

#endif  // JOYSTICK_CONTROL_H_