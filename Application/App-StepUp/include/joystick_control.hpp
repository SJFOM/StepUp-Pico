#ifndef JOYSTICK_CONTROL_H_
#define JOYSTICK_CONTROL_H_

// pico-sdk
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`
#include "hardware/adc.h"

// Common
#include "../../../Common/utils.h"

// Control libraries
#include "../../../Interfaces/ControlInterface.hpp"

#define ADC_PIN_JOYSTICK_X (26U)
#define ADC_PIN_JOYSTICK_Y (27U)

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
};

#endif  // JOYSTICK_CONTROL_H_