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
    NOTIFY_FUNC_MAX_COUNT
};

static const char *ControllerNotificationString
    [ControllerNotification::NOTIFY_FUNC_MAX_COUNT] = {"NOTIFY_BOOT",
                                                       "NOTIFY_INFO",
                                                       "NOTIFY_DATA",
                                                       "NOTIFY_WARN",
                                                       "NOTIFY_ERROR"};

class ControlInterface
{
public:
    virtual bool init() = 0;
    virtual void deinit() = 0;

    /**
     * @brief Control the functional state of the peripheral
     *
     * @param enable_disable
     */
    virtual void enableFunctionality(bool enable_disable)
    {
        m_is_enabled = enable_disable;
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

protected:
private:
    bool m_init_success;
    bool m_is_enabled;
};

#endif  // CONTROL_INTERFACE_H_