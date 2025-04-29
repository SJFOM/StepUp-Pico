/**
 * General utility functions
 *
 * Adapted from:
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.4.1
 * @licence   MIT
 *
 */

#include "utils.h"
#include <cstdarg>  // For va_list, va_start, va_end
#include <cstdio>
#include <string>

using std::string;
using std::vector;

namespace Utils
{

    /*************************
     * LOGGING UTILS - BEGIN *
     *************************/
    /**
     * @brief Convert a multi-line string into an array of lines,
     *        split at the specified separator string, eg. `\r\n`.
     *
     * @param ml_str:    The multi-line string.
     * @param separator: The line-separator string.
     *
     * @retval The lines as a vector.
     */
    vector<string> split_to_lines(string ml_str, string separator)
    {
        vector<string> result;
        while (ml_str.length())
        {
            const int index = ml_str.find(separator);
            if (index != string::npos)
            {
                result.push_back(ml_str.substr(0, index));
                ml_str = ml_str.substr(index + separator.length());
            }
            else
            {
                result.push_back(ml_str);
                break;
            }
        }

        return result;
    }

    /**
     * @brief Get a specific line from a multi-line string.
     *
     * @param ml_str:    The multi-line string.
     * @param want_line: The required line (0 indexed).
     *
     * @retval The requested line, otherwise an empty string.
     */
    string split_msg(string ml_str, uint32_t want_line)
    {
        const vector<string> lines = split_to_lines(ml_str);
        for (uint32_t i = 0; i < lines.size(); ++i)
        {
            if (i == want_line) return lines[i];
        }
        return "";
    }

    /**
     * @brief Get a number from the end of a +CMTI line from the modem.
     *
     * @param line: The target line.
     *
     * @retval A pointer to the start of the number, or `null`.
     */
    string get_sms_number(string line)
    {
        return get_field_value(line, 1);
    }

    /**
     * @brief A value from a sequence of comma-separated values.
     *
     * @param line:         The source line.
     * @param field_number: The request value.
     *
     * @retval The value as a string, otherwise an empty string.
     */
    string get_field_value(string line, uint32_t field_number)
    {
        const vector<string> result = split_to_lines(line, ",");
        if (result.size() > field_number) return result[field_number];
        return "";
    }

    /**
     * @brief Convert a 16-bit int (to cover decimal range 0-9999) to
     *        its BCD equivalent.
     *
     * @param base: The input integer.
     *
     * @retval The BCD encoding of the input.
     */
    uint32_t bcd(uint32_t base)
    {
        if (base > 9999) base = 9999;
        for (uint32_t i = 0; i < 16; ++i)
        {
            base = base << 1;
            if (i == 15) break;
            if ((base & 0x000F0000) > 0x0004FFFF) base += 0x00030000;
            if ((base & 0x00F00000) > 0x004FFFFF) base += 0x00300000;
            if ((base & 0x0F000000) > 0x04FFFFFF) base += 0x03000000;
            if ((base & 0xF0000000) > 0x4FFFFFFF) base += 0x30000000;
        }

        return (base >> 16) & 0xFFFF;
    }

    /**
     * @brief Convert a string to uppercase.
     *
     * @param base: The input string.
     *
     * @retval An uppercase string.
     */
    string uppercase(string base)
    {
        // string result;
        std::transform(base.begin(), base.end(), base.begin(), ::toupper);
        return base;
    }

    /**
     * @brief Output basic device info.
     */
    void log_device_info(void)
    {
        printf("[INFO] App: %s %s (%s)\n", APP_NAME, APP_VERSION, BUILD_NUM);
    }

    /**
     * @brief Generate and print an INFO message from a supplied string.
     *
     * @param msg: The base message to which `[INFO]` will be prefixed.
     */
    void log_info(const string msg)
    {
        printf("[INFO] %s%\n", msg.c_str());
    }

    /**
     * @brief Generate and print a DEBUG message from a supplied string.
     *
     * @param msg: The base message to which `[DBUG]` will be prefixed (kept to
     * 4 letters)
     */
    void log_debug(const string msg)
    {
#ifdef DEBUG
        printf("[DBUG] %s%\n", msg.c_str());
#endif
    }

    /**
     * @brief Generate and print a WARNing message from a supplied string.
     *
     * @param msg: The base message to which `[WARN]` will be prefixed.
     */
    void log_warn(const string msg)
    {
        printf("[WARN] %s\n", msg.c_str());
    }

    /**
     * @brief Generate and print an ERROR message from a supplied string.
     *
     * @param msg: The base message to which `[ERROR]` will be prefixed.
     */
    void log_error(const string msg)
    {
        printf("[ERROR] %s\n", msg.c_str());
        // printf("[POWER] Powering down...\n");
        // PowerControl::powerDown();
        while (true)
        {
            ;
        }
    }

    /**************************
     * LOGGING UTILS - FINISH *
     **************************/

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

}  // namespace Utils
