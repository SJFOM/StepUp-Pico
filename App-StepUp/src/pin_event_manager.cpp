/**
 * @file pin_event_manager.cpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2025-03-23
 *
 * @copyright Copyright (c) 2025
 * @license   MIT
 *
 */

#include "../include/pin_event_manager.hpp"

void pin_event_interrupt_callback();

PinEventManager *PinEventManager::s_p_pin_event_manager[] = {nullptr};
uint8_t PinEventManager::s_pin_interrupt_count = 0;

PinEventManager::PinEventManager(uint8_t pin,
                                 uint32_t event_type,
                                 uint32_t long_press_duration_in_ms)
    : m_pin(pin),
      m_event_type(event_type),
      m_long_press_duration_in_ms(long_press_duration_in_ms)
{
    if (s_pin_interrupt_count < s_max_pin_interrupt_count)
    {
        if (m_long_press_duration_in_ms == 0U)  // Default, not set
        {
            m_pin_event_timeout_ms = s_pin_debounce_default_delay_time_ms;
        }
        else
        {
            m_pin_event_timeout_ms = m_long_press_duration_in_ms;
        }

        s_p_pin_event_manager[s_pin_interrupt_count] = this;
        s_pin_interrupt_count++;
    }
    else
    {
        LOG_ERROR("PinEventManager: Too many instances created!");
    }
};

PinEventManager::~PinEventManager()
{
    deinit();
}

bool PinEventManager::init()
{
    bool init_success = true;
    // Based on the event type, we need to declare the expected GPIO states
    // which determine whether the pin is "active" vs "idle"
    switch (m_event_type)
    {
        case (gpio_irq_level::GPIO_IRQ_LEVEL_HIGH):
        case (gpio_irq_level::GPIO_IRQ_EDGE_RISE):
        {
            m_active_state_is_when_pin_is_high = true;
            break;
        }
        case (gpio_irq_level::GPIO_IRQ_EDGE_FALL):
        case (gpio_irq_level::GPIO_IRQ_LEVEL_LOW):
        {
            m_active_state_is_when_pin_is_high = false;
            break;
        }
        default:
            init_success = false;
    }

    if (init_success)
    {
        // We should be able to use the same interrupt handler method for all
        // interrupt events as these are shared anyway (ORed together in RP2040
        // hardware)
        gpio_add_raw_irq_handler(m_pin, &pin_event_interrupt_callback);
        gpio_set_irq_enabled(m_pin, GPIO_IRQ_EDGE_FALL, true);

        // Need to confirm the BANK0 IRQ is enabled (only needed once)
        if (!irq_is_enabled(IO_IRQ_BANK0))
        {
            irq_set_enabled(IO_IRQ_BANK0, true);
        }
    }

    return init_success;
}

void PinEventManager::deinit()
{
    // TODO: Remove interrupts
    //  m_init_success = false;
}

int64_t pin_event_debounce_timer_callback(alarm_id_t id, void *user_data)
{
    for (uint8_t count = 0; count < PinEventManager::s_pin_interrupt_count;
         count++)
    {
        PinEventManager *p_irq_instance =
            PinEventManager::s_p_pin_event_manager[count];

        if (p_irq_instance->isDebounceTimerActive())
        {
            // TODO: Check timeout has elapsed for given instance (as we may
            // have more than one instance active at a given time)
            if (p_irq_instance->m_active_state_is_when_pin_is_high ==
                gpio_get(p_irq_instance->m_pin))
            {
                p_irq_instance->incrementPinEventOccured();
                LOG_INFO("TIMEOUT!");
            }

            p_irq_instance->setDebounceTimerActive(false);

            // Re-enable the interrupt
            gpio_set_irq_enabled(p_irq_instance->m_pin,
                                 (p_irq_instance->m_event_type),
                                 true);

            p_irq_instance->m_debounce_timer_id = 0;
        }
    }
    return 0;
}

void pin_event_interrupt_callback()
{
    for (uint8_t count = 0; count < PinEventManager::s_pin_interrupt_count;
         count++)
    {
        PinEventManager *p_irq_instance =
            PinEventManager::s_p_pin_event_manager[count];

        if (gpio_get_irq_event_mask(p_irq_instance->m_pin) &
            (p_irq_instance->m_event_type))
        {
            gpio_acknowledge_irq(p_irq_instance->m_pin,
                                 (p_irq_instance->m_event_type));
            // Disable interrupt until debounce timer has elapsed
            // gpio_set_irq_enabled(p_irq_instance->m_pin,
            //                      (p_irq_instance->m_event_type),
            //                      false);

            // Cancel the alarm if it exists, do nothing otherwise
            if (cancel_alarm(p_irq_instance->m_debounce_timer_id))
            {
                LOG_INFO("CNCL");
            }

            LOG_INFO("INT!");
            p_irq_instance->setDebounceTimerActive(true);
            p_irq_instance->m_debounce_timer_id =
                add_alarm_in_ms(p_irq_instance->m_pin_event_timeout_ms,
                                pin_event_debounce_timer_callback,
                                NULL,
                                false);
            // }
        }
    }
}
