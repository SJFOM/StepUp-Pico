#ifndef JOYSTICK_CONTROL_H_
#define JOYSTICK_CONTROL_H_

// pico-sdk
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`
#include "hardware/adc.h"

// Common
#include "../../../Common/utils.h"

// Control libraries
#include "../../../Interfaces/ControlInterface.hpp"

// ADC Joystick pins
#define ADC_PIN_JOYSTICK_X (26U)
#define ADC_PIN_JOYSTICK_Y (27U)

// ADC homing position allowable threshold values
#define ADC_MIDWAY_VALUE_RAW (1<<6U)
#define ADC_HOME_THRESHOLD_RAW (100U)
#define ADC_LOWER_HOME_THRESHOLD_RAW (ADC_MIDWAY_VALUE_RAW + ADC_HOME_THRESHOLD_RAW)
#define ADC_UPPER_HOME_THRESHOLD_RAW (ADC_MIDWAY_VALUE_RAW + ADC_HOME_THRESHOLD_RAW)

struct JoystickPosition
{
    int16_t x, y;
    uint16_t x_offset, y_offset;
};

class JoystickControl : public ControlInterface
{
public:
    JoystickControl();
    ~JoystickControl();
    bool init();
    void deinit();
    void processJob(uint32_t tick_count);

protected:

private:
    bool m_init_success;
    struct JoystickPosition joystick_pos;
};

#endif  // JOYSTICK_CONTROL_H_