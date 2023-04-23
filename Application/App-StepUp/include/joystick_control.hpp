#ifndef JOYSTICK_CONTROL_H_
#define JOYSTICK_CONTROL_H_

// pico-sdk
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`
#include "hardware/adc.h"
#include "hardware/gpio.h"

// pin includes
#include "pins_definitions.h"

// Common
#include "../../../Common/utils.h"

// Control libraries
#include "../../../Interfaces/ControlInterface.hpp"

// ADC homing position allowable threshold values
#define ADC_MIDWAY_VALUE_RAW (1<<11U) // 12 bit ADC, take mid-way value (half)
#define ADC_HOME_THRESHOLD_RAW (100U)
#define ADC_LOWER_HOME_THRESHOLD_RAW (ADC_MIDWAY_VALUE_RAW + ADC_HOME_THRESHOLD_RAW)
#define ADC_UPPER_HOME_THRESHOLD_RAW (ADC_MIDWAY_VALUE_RAW + ADC_HOME_THRESHOLD_RAW)

struct JoystickPosition
{
    int16_t x_pos, y_pos;
    uint16_t x_offset, y_offset;
};

enum JoystickState
{
    JOYSTICK_STATE_IDLE = 0,
    JOYSTICK_STATE_LOW,
    JOYSTICK_STATE_MID,
    JOYSTICK_STATE_HIGH,
};
struct JoystickData
{
    JoystickPosition x_stage, y_stage;
    enum JoystickState state;
    bool button_is_pressed;
};

class JoystickControl : public ControlInterface
{
public:
    JoystickControl();
    ~JoystickControl();
    bool init();
    void deinit();
    void processJob(uint32_t tick_count);
    enum JoystickState getJoystickState();

protected:

private:
    bool m_init_success;
    struct JoystickData m_joystick;
};

#endif  // JOYSTICK_CONTROL_H_