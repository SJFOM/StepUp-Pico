#ifndef _HARDWARE_GPIO_H_
#define _HARDWARE_GPIO_H_

#include <stdint.h>
#include "../types.h"

// #ifdef __cplusplus
// extern "C"
// {
// #endif

#define uart1 1U

void sleep_ms(unsigned long);
unsigned long get_absolute_time()
{
    return 0;
}

unsigned uart_init(unsigned id, unsigned baud_rate)
{
    return 0;
}
void uart_deinit(unsigned id);

void uart_read_blocking(unsigned id,
                        unsigned char *data_buf,
                        unsigned data_len);

void uart_write_blocking(unsigned id,
                         unsigned char *data_buf,
                         unsigned data_len);

unsigned long to_ms_since_boot(unsigned absolute_time)
{
    return 0;
}

enum gpio_function
{
    GPIO_FUNC_XIP = 0,
    GPIO_FUNC_SPI = 1,
    GPIO_FUNC_UART = 2,
    GPIO_FUNC_I2C = 3,
    GPIO_FUNC_PWM = 4,
    GPIO_FUNC_SIO = 5,
    GPIO_FUNC_PIO0 = 6,
    GPIO_FUNC_PIO1 = 7,
    GPIO_FUNC_GPCK = 8,
    GPIO_FUNC_USB = 9,
    GPIO_FUNC_NULL = 0x1f,
};

#define GPIO_OUT 1
#define GPIO_IN  0

enum gpio_irq_level
{
    GPIO_IRQ_LEVEL_LOW = 0x1u,
    GPIO_IRQ_LEVEL_HIGH = 0x2u,
    GPIO_IRQ_EDGE_FALL = 0x4u,
    GPIO_IRQ_EDGE_RISE = 0x8u,
};

typedef void (*gpio_irq_callback_t)(uint gpio, uint32_t events);

enum gpio_override
{
    GPIO_OVERRIDE_NORMAL =
        0,  ///< peripheral signal selected via \ref gpio_set_function
    GPIO_OVERRIDE_INVERT = 1,  ///< invert peripheral signal selected via
                               ///< \ref gpio_set_function
    GPIO_OVERRIDE_LOW = 2,     ///< drive low/disable output
    GPIO_OVERRIDE_HIGH = 3,    ///< drive high/enable output
};

enum gpio_slew_rate
{
    GPIO_SLEW_RATE_SLOW = 0,  ///< Slew rate limiting enabled
    GPIO_SLEW_RATE_FAST = 1   ///< Slew rate limiting disabled
};

enum gpio_drive_strength
{
    GPIO_DRIVE_STRENGTH_2MA = 0,  ///< 2 mA nominal drive strength
    GPIO_DRIVE_STRENGTH_4MA = 1,  ///< 4 mA nominal drive strength
    GPIO_DRIVE_STRENGTH_8MA = 2,  ///< 8 mA nominal drive strength
    GPIO_DRIVE_STRENGTH_12MA = 3  ///< 12 mA nominal drive strength
};

// ----------------------------------------------------------------------------
// Pad Controls + IO Muxing
// ----------------------------------------------------------------------------
// Declarations for gpio.c

void gpio_set_function(uint gpio, enum gpio_function fn);

enum gpio_function gpio_get_function(uint gpio);

void gpio_set_pulls(uint gpio, bool up, bool down);

void gpio_pull_up(uint gpio) {}

bool gpio_is_pulled_up(uint gpio)
{
    return false;
}

void gpio_pull_down(uint gpio) {}

bool gpio_is_pulled_down(uint gpio)
{
    return false;
}

void gpio_disable_pulls(uint gpio);
void gpio_set_irqover(uint gpio, uint value);

void gpio_set_outover(uint gpio, uint value);

void gpio_set_inover(uint gpio, uint value);

void gpio_set_oeover(uint gpio, uint value);

void gpio_set_input_enabled(uint gpio, bool enabled);

void gpio_set_input_hysteresis_enabled(uint gpio, bool enabled);

bool gpio_is_input_hysteresis_enabled(uint gpio);

void gpio_set_slew_rate(uint gpio, enum gpio_slew_rate slew);

enum gpio_slew_rate gpio_get_slew_rate(uint gpio);

void gpio_set_drive_strength(uint gpio, enum gpio_drive_strength drive);

enum gpio_drive_strength gpio_get_drive_strength(uint gpio);

void gpio_set_irq_enabled(uint gpio, uint32_t events, bool enabled);

void gpio_set_irq_enabled_with_callback(uint gpio,
                                        uint32_t events,
                                        bool enabled,
                                        gpio_irq_callback_t callback);

void gpio_set_dormant_irq_enabled(uint gpio, uint32_t events, bool enabled);

void gpio_acknowledge_irq(uint gpio, uint32_t events);

void gpio_init(uint gpio);

void gpio_deinit(uint gpio);

void gpio_init_mask(uint gpio_mask);
// ----------------------------------------------------------------------------
// Input
// ----------------------------------------------------------------------------

bool gpio_get(uint gpio)
{
    return false;
}

uint32_t gpio_get_all(void)
{
    return 0;
}

// ----------------------------------------------------------------------------
// Output
// ----------------------------------------------------------------------------

void gpio_set_mask(uint32_t mask) {}

void gpio_clr_mask(uint32_t mask) {}

void gpio_xor_mask(uint32_t mask) {}

void gpio_put_masked(uint32_t mask, uint32_t value) {}

void gpio_put_all(uint32_t value) {}

void gpio_put(uint gpio, bool value) {}

bool gpio_get_out_level(uint gpio)
{
    return true;
}

// ----------------------------------------------------------------------------
// Direction
// ----------------------------------------------------------------------------

void gpio_set_dir_out_masked(uint32_t mask) {}

void gpio_set_dir_in_masked(uint32_t mask) {}

void gpio_set_dir_masked(uint32_t mask, uint32_t value) {}

void gpio_set_dir_all_bits(uint32_t values) {}

void gpio_set_dir(uint gpio, bool out) {}

bool gpio_is_dir_out(uint gpio)
{
    return true;
}

uint gpio_get_dir(uint gpio)
{
    return true;
}

extern void gpio_debug_pins_init(void);

// #ifdef __cplusplus
// }
// #endif

#endif  // _GPIO_H_
