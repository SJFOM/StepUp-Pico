#ifndef JOYSTICK_CONTROL_H_
#define JOYSTICK_CONTROL_H_

// pico-sdk
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`

// pin includes
#include "pins_definitions.h"

// Logging utilities
#include "utils.h"

// Control libraries
#include "../../../Interfaces/ControlInterface.hpp"

// ADC ENOB ~= 8.7 - See Section 4.9.3 of datasheet
#define ADC_ENOB_MASK (0xFF8)  // top 9 MSB's are valid

// ADC homing position allowable threshold values
#define ADC_MIDWAY_VALUE_RAW \
    (1 << 11U)  // 12 bit ADC, take mid-way value (half)
#define ADC_HOME_THRESHOLD_RAW (100U)
#define ADC_LOWER_HOME_THRESHOLD_RAW \
    (ADC_MIDWAY_VALUE_RAW - ADC_HOME_THRESHOLD_RAW)
#define ADC_UPPER_HOME_THRESHOLD_RAW \
    (ADC_MIDWAY_VALUE_RAW + ADC_HOME_THRESHOLD_RAW)

#define JOYSTICK_THRESHOLD_UPPER (1000)
#define JOYSTICK_THRESHOLD_LOWER (-JOYSTICK_THRESHOLD_UPPER)

// 12-bit conversion, assume max value == ADC_VREF == 3.3 V
#define ADC_TO_VOLTAGE_CONVERSION_FACTOR (3.3f / (1 << 12))

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
    enum JoystickState getJoystickState();
    struct JoystickData getJoystickData();

protected:
private:
    bool m_init_success;
    struct JoystickData m_joystick;
};

#endif  // JOYSTICK_CONTROL_H_