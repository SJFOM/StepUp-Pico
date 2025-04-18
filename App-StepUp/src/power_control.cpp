/**
 * @file power_control.cpp
 * @author Sam (@SJFOM)
 * @brief Implementation of the PowerControl class
 * @details ...
 * @version 0.1
 * @date 2025-04-17
 *
 * @copyright Copyright (c) 2025
 * @license MIT
 */

#include "../include/power_control.hpp"
#include "PicoUtils.h"

PowerControl::PowerControl() {}

PowerControl::~PowerControl()
{
    deinit();
}

bool PowerControl::init()
{
    return true;
}

void PowerControl::deinit()
{
    ;
}

void PowerControl::powerDown()
{
    ;
}

enum ControllerState PowerControl::processJob(uint32_t tick_count)
{
    enum ControllerState power_state = ControllerState::STATE_IDLE;

    // if (!m_init_success)
    // {
    //     LOG_ERROR("PowerControl not initialized.");
    //     return STATE_IDLE;
    // }

    return power_state;
}
