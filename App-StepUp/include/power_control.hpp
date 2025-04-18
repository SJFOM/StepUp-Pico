/**
 * @file power_control.hpp
 * @author Sam (@SJFOM)
 * @brief Library for monitoring voltage levels and reporting on them when they
 * are detected to be out of bounds
 * @version 0.1
 * @date 2025-04-17
 *
 * @copyright Copyright (c) 2025
 * @license MIT
 */

#ifndef POWER_CONTROL_H_
#define POWER_CONTROL_H_

// Generic c includes
#include <string>

// pico-sdk
#include "hardware/adc.h"
#include "pico/stdlib.h"

// Control libraries
#include "ControlInterface.hpp"

class PowerControl : public ControlInterface
{
public:
    PowerControl();
    ~PowerControl();

    bool init() override;
    void deinit() override;
    enum ControllerState processJob(uint32_t tick_count) override;

    static void powerDown();

protected:
private:
};

#endif  // POWER_CONTROL_H_