#ifndef JOYSTICK_CONTROL_H_
#define JOYSTICK_CONTROL_H_

// pico-sdk
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`

// Common
#include "../../../Common/utils.h"

#include "../../../Interfaces/ControlInterface.hpp"

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