/**
 * General utility functions
 *
 * Adapted from:
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.4.1
 * @licence   MIT
 *
 */
#ifndef UTILS_HEADER
#define UTILS_HEADER

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>
// Pico SDK
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`

using std::string;
using std::vector;

// ADC ENOB ~= 8.7 - See Section 4.9.3 of datasheet
#define ADC_ENOB_MASK (0xFF8)  // top 9 MSB's are valid

// ADC homing position allowable threshold values
#define ADC_MIDWAY_VALUE_RAW \
    (1 << 11U)  // 12 bit ADC, take mid-way value (half)

// 12-bit conversion, assume max value == ADC_VREF == 3.3 V
#define ADC_TO_VOLTAGE_CONVERSION_FACTOR (3.3f / (1 << 12))

// Logging macros
#define LOG_INFO(x)   Utils::log_info(x)
#define LOG_DEBUG(x)  Utils::log_debug(x)
#define LOG_WARN(x)   Utils::log_warn(x)
#define LOG_ERROR(x)  Utils::log_error(x)
#define LOG_DATA(...) Utils::log_data(__VA_ARGS__)

/*
 * PROTOTYPES
 */
namespace Utils
{
    /*************************
     * LOGGING UTILS - BEGIN *
     *************************/
    vector<string> split_to_lines(string str, string sep = "\r\n");
    string split_msg(string msg, uint32_t want_line);
    string get_sms_number(string line);
    string get_field_value(string line, uint32_t field_number);
    string uppercase(string base);
    uint32_t bcd(uint32_t base);
    void log_device_info(void);
    void log_info(const string msg);
    void log_debug(const string msg);
    void log_warn(const string msg);
    void log_error(const string msg);

    /**
     * @brief Generate and print a formatted log message with data for a single
     * argument.
     *
     * @param format: The format string (e.g., "Voltage is low: %.2fV").
     * @param args: The values to format and log.
     */
    template <typename Args>
    void log_data(const char *format, Args args)
    {
        char formatted_message[128];
        snprintf(formatted_message, sizeof(formatted_message), format, args);
        printf("[DATA] %s\n", formatted_message);
    }

    // /**
    //  * @brief Generate and print a formatted log message with data for more
    //  than
    //  * one argument.
    //  *
    //  * @param format: The format string (e.g., "Voltage is low: %.2fV").
    //  * @param ...args: The values to format and log.
    //  */
    template <typename... Args>
    void log_data(const char *format, Args... args)
    {
        char formatted_message[128];
        snprintf(formatted_message, sizeof(formatted_message), format, args...);
        printf("[DATA] %s\n", formatted_message);
    }

    /**************************
     * LOGGING UTILS - FINISH *
     **************************/

    /************************
     * NUMBER UTILS - BEGIN *
     ************************/

    template <typename T>
    bool isNumberWithinBounds(T value, T lower_bound, T upper_bound)
    {
        // Static assert will fail if T is not a numeric type. This is evaluated
        // at compile time vs run-time.
        static_assert(std::is_arithmetic<T>::value, "T must be a numeric type");

        // Handle floating-point precision issues
        if constexpr (std::is_floating_point<T>::value)
        {
            constexpr T epsilon =
                static_cast<T>(1e-6);  // Adjust tolerance as needed
            return (value <= upper_bound + epsilon) &&
                   (value >= lower_bound - epsilon);
        }

        // For integral types, perform direct comparison
        return (bool)(value <= upper_bound) && (value >= lower_bound);
    }

    /*************************
     * NUMBER UTILS - FINISH *
     *************************/

    /*********************
     * ADC UTILS - BEGIN *
     *********************/
    bool isADCInitialised();

    uint16_t getValidADCResultRaw(uint8_t adc_channel);
    float getValidADCResultVolts(uint8_t adc_channel);

    /**********************
     * ADC UTILS - FINISH *
     **********************/

    /*********************
     * PWM UTILS - BEGIN *
     *********************/
    uint16_t configurePWMPin(uint pwm_pin);
    void setPWMFrequency(uint pwm_pin,
                         uint16_t pwm_freq_in_hz,
                         uint8_t duty_cycle_percentage = 50U);

    /**********************
     * PWM UTILS - FINISH *
     **********************/

}  // namespace Utils

#endif  // UTILS_HEADER
