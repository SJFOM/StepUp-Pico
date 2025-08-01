/**
 * @file ControlInterface.hpp
 * @author Sam (@SJFOM)
 * @brief Interface for controlling the state of a peripheral
 * @details This interface exposes common ways to control functionalities of a
 * peripheral. Through its use as a parent class, it enables consistent design
 * of child peripherals which can be processed via a master control loop
 * (usually in main.cpp).
 * @version 0.1
 * @date 2023-02-11
 *
 * @copyright Copyright (c) 2023
 * @license MIT
 */

#ifndef CONTROL_INTERFACE_H_
#define CONTROL_INTERFACE_H_

#include "PicoUtils.h"

constexpr uint8_t CX_CONTROL_INTERFACE_MAX_COUNT = 20U;

enum ControllerState
{
    STATE_IDLE = 0,
    STATE_READY,
    STATE_BUSY,
    STATE_NEW_DATA,
    STATE_COUNT,
};

static const char *ControllerStateString[ControllerState::STATE_COUNT] = {
    "STATE_IDLE",
    "STATE_READY",
    "STATE_BUSY",
    "STATE_NEW_DATA",
};

enum ControllerNotification
{
    NOTIFY_BOOT = 0U,
    NOTIFY_INFO = 1U,
    NOTIFY_DATA = 2U,
    NOTIFY_WARN = 3U,
    NOTIFY_ERROR = 4U,
    NOTIFY_POWER_DOWN = 5U,
    NOTIFY_FUNC_MAX_COUNT
};

static const char *ControllerNotificationString
    [ControllerNotification::NOTIFY_FUNC_MAX_COUNT] = {"NOTIFY_BOOT",
                                                       "NOTIFY_INFO",
                                                       "NOTIFY_DATA",
                                                       "NOTIFY_WARN",
                                                       "NOTIFY_ERROR",
                                                       "NOTIFY_POWER_DOWN"};

class ControlInterface
{
public:
    ControlInterface();
    virtual ~ControlInterface();

    virtual bool init() = 0;
    virtual void deinit() = 0;

    /**
     * @brief Control the functional state of the peripheral
     *
     * @param enable_disable
     */
    virtual void enableFunctionality(bool enable_disable)
    {
        m_latest_activity_change_timestamp_ms =
            PicoUtils::getCurrentTimestampMs();
        m_is_enabled = enable_disable;
        enablePeripheralDriver(enable_disable);
    }

    /**
     * @brief Get the Last Activate Timestamp ms object
     *
     * @return uint32_t
     */
    virtual uint32_t getLatestActivityChangeTimestampMs()
    {
        return m_latest_activity_change_timestamp_ms;
    }

    /**
     * @brief Get peripherals functional state
     *
     * @return true if peripheral can be used
     * @return false if peripheral should not yet be used
     */
    virtual bool isFunctionalityEnabled()
    {
        return m_is_enabled;
    }
    virtual enum ControllerState processJob(uint32_t tick_count) = 0;

    /**
     * @brief ...
     *
     * @details Static access function to poll all control interfaces for the
     * latest active timestamp information
     *
     * @return uint32_t
     */
    static uint32_t getLastTimeControlPeripheralActivityWasUpdatedMs();

    static ControlInterface
        *sp_control_interfaces[CX_CONTROL_INTERFACE_MAX_COUNT];
    static uint8_t s_control_interfaces_count;

protected:
    bool m_init_success, m_is_enabled;

private:
    uint8_t m_control_interface_index;
    uint32_t m_latest_activity_change_timestamp_ms;

    static uint32_t s_most_recent_deactivate_timestamp_ms;

    /**
     * @brief Enable or disable the peripheral driver
     *
     * @details This is the default implementation for enabling the peripheral.
     * It is called via the public facing wrapper method "enableFunctionality"
     * to append meta-data such as enable/disable timestamp information
     *
     * @param enable_disable
     */
    virtual void enablePeripheralDriver(bool enable_disable)
    {
        m_is_enabled = enable_disable;
    }
};

#endif  // CONTROL_INTERFACE_H_