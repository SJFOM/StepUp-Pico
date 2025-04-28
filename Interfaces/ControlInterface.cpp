/**
 * @file ControlInterface.cpp
 * @author Sam (@SJFOM)
 * @brief Interface for global access to inhereted classes
 * @details This interface exposes some class implementation member variables
 * through static access functions to allow for global access.
 * @details ...
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2025
 * @license MIT
 */

#include "ControlInterface.hpp"

ControlInterface
    *ControlInterface::sp_control_interfaces[CX_CONTROL_INTERFACE_MAX_COUNT];
uint8_t ControlInterface::s_control_interfaces_count;

ControlInterface::ControlInterface()
{
    for (uint8_t index = 0; index < CX_CONTROL_INTERFACE_MAX_COUNT; index++)
    {
        if (sp_control_interfaces[index] == nullptr)
        {
            sp_control_interfaces[index] = this;
            m_control_interface_index = index;
            break;
        }
        s_control_interfaces_count++;
    }
}

ControlInterface::~ControlInterface()
{
    sp_control_interfaces[m_control_interface_index] = nullptr;
    s_control_interfaces_count--;
}

uint32_t ControlInterface::getLastTimeControlPeripheralWasUsedMs()
{
    uint32_t most_recent_timestamp = 0;
    for (uint8_t index = 0; index < s_control_interfaces_count; index++)
    {
        if (sp_control_interfaces[index] != nullptr)
        {
            uint32_t timestamp =
                sp_control_interfaces[index]->getLastActivateTimestampMs();
            if (timestamp > most_recent_timestamp &&
                !sp_control_interfaces[index]->isFunctionalityEnabled())
            {
                most_recent_timestamp = timestamp;
            }
        }
    }
    return most_recent_timestamp;
}