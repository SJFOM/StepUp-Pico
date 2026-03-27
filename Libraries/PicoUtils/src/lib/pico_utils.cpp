/**
 * @file pico_utils.cpp
 * @brief Pico specific utility functions
 * @details Library for Pico to provide common access methods which using Pico
 * specific hardware access functions
 * @version 0.1
 * @date 2025-07-18
 *
 * @copyright Copyright (c) 2025
 * @license MIT
 */

#include "pico_utils.h"
#include <cstdarg>  // For va_list, va_start, va_end
#include <cstdio>
#include <string>

using std::string;
using std::vector;

namespace PicoUtils
{
    /***************************
     * TIMESTAMP UTILS - START *
     ***************************/
    uint32_t getCurrentTimestampMs()
    {
        // Get the current time in milliseconds since boot
        absolute_time_t current_time = get_absolute_time();
        uint32_t current_timestamp_ms =
            to_ms_since_boot(current_time) % UINT32_MAX;
        return current_timestamp_ms;
    }

    uint32_t getElapsedTimeMs(uint32_t start_time_ms)
    {
        // Get the current time in milliseconds since boot
        absolute_time_t current_time = get_absolute_time();
        uint32_t current_timestamp_ms =
            to_ms_since_boot(current_time) % UINT32_MAX;

        // Calculate the elapsed time in milliseconds
        uint32_t elapsed_time_ms =
            (current_timestamp_ms - start_time_ms) % UINT32_MAX;
        return elapsed_time_ms;
    }

    /****************************
     * TIMESTAMP UTILS - FINISH *
     ****************************/

    /*********************
     * ADC UTILS - BEGIN *
     *********************/
    bool isADCInitialised()
    {
        return (bool)(adc_hw->cs & ADC_CS_READY_BITS);
    }

    uint16_t getValidADCResultRaw(uint8_t adc_channel)
    {
        // Ensure valid ADC channels are being used (0 -> 3)
        assert(Utils::isNumberWithinBounds<uint8_t>(adc_channel, 0U, 3U));

        adc_select_input(adc_channel);
        return (uint16_t)(adc_read() & ADC_ENOB_MASK);
    }

    float getValidADCResultVolts(uint8_t adc_channel)
    {
        return (float)((float)getValidADCResultRaw(adc_channel) *
                       ADC_TO_VOLTAGE_CONVERSION_FACTOR);
    }
    /**********************
     * ADC UTILS - FINISH *
     **********************/

    /*********************
     * PWM UTILS - BEGIN *
     *********************/

    uint16_t configurePWMPin(uint pwm_pin)
    {
        // Ensure valid PWM pins are being used (0 -> 29)
        assert(pwm_pin < NUM_BANK0_GPIOS);

        // Configure the led pins and direction
        gpio_set_function(pwm_pin, GPIO_FUNC_PWM);

        // Find out which PWM slice is connected to pwm_pin
        uint16_t pwm_slice_num = pwm_gpio_to_slice_num(pwm_pin);

        // Get some sensible defaults for the slice configuration. By
        // default, the counter is allowed to wrap over its maximum range (0
        // to 2**16-1)
        pwm_config config = pwm_get_default_config();

        // Apply the configuration to the PWM slice
        pwm_init(pwm_slice_num, &config, true);

        return pwm_slice_num;
    }

    void setPWMFrequency(uint pwm_pin,
                         uint16_t pwm_freq_in_hz,
                         uint8_t duty_cycle_percentage)
    {
        uint pwm_slice_num = pwm_gpio_to_slice_num(pwm_pin);

        if (pwm_freq_in_hz > 0)
        {
            check_slice_num_param(
                pwm_slice_num);  // Check slice number is valid

            uint32_t system_clock = SYS_CLK_HZ;
            uint32_t divider_16 = system_clock / pwm_freq_in_hz / 4096 +
                                  (system_clock % (pwm_freq_in_hz * 4096) != 0);

            if (divider_16 / 16 == 0)
            {
                divider_16 = 16;
            }

            // Calculate the pwm wrap value, which is the maximum value of the
            // counter. The counter will count from 0 to wrap, and then
            // wrap around to 0 again.
            uint32_t wrap = system_clock * 16 / divider_16 / pwm_freq_in_hz - 1;

            pwm_set_clkdiv_int_frac(pwm_slice_num,
                                    divider_16 / 16,
                                    divider_16 & 0xF);
            pwm_set_wrap(pwm_slice_num, wrap);
            pwm_set_gpio_level(pwm_pin, wrap * duty_cycle_percentage / 100);

            // Enable the PWM output
            pwm_set_enabled(pwm_slice_num, true);
        }
        else
        {
            // Disable the PWM output
            pwm_set_enabled(pwm_slice_num, false);
        }
    }

    /**********************
     * PWM UTILS - FINISH *
     **********************/

}  // namespace PicoUtils
