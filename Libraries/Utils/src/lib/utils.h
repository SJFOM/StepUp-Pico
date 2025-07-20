/**
 * @file utils.h
 * @author Sam (@SJFOM)
 * @brief Util methods for the Pico SDK
 * @version 0.1
 * @date 2025-04-18
 *
 * @copyright Copyright (c) 2024
 * @license   MIT
 *
 *
 * Adapted from:
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.4.1
 * @licence   MIT
 *
 */

#ifndef UTILS_HEADER_H_
#define UTILS_HEADER_H_

#include <assert.h>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

using std::string;
using std::vector;

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
    // Constants local to the namespace
    constexpr static float csx_epsilon = 1e-6f;

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
     * NUMBER UTILS - START *
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
            constexpr T epsilon = static_cast<T>(
                Utils::csx_epsilon);  // Adjust tolerance as needed
            return (value <= upper_bound + epsilon) &&
                   (value >= lower_bound - epsilon);
        }

        // For integral types, perform direct comparison
        return (bool)(value <= upper_bound) && (value >= lower_bound);
    }

    class ExponentialMovingAverage
    {
    public:
        ExponentialMovingAverage(float alpha = 0.1f)
            : m_alpha(alpha),
              m_accumulator(0)
        {
            assert(Utils::isNumberWithinBounds<float>(m_alpha, 0.f, 1.0f));
        };

        ~ExponentialMovingAverage()
        {
            m_accumulator = 0.f;
        }

        void push(float new_value)
        {
            // The closer (1.0 - alpha) is to 1.0, the longer the effect of
            // previous numbers hangs around, and the less impact each new
            // number has. Covnersely, the closer alpha is to 1.0, the faster
            // the moving average updates in response to new values.
            m_accumulator =
                (m_alpha * new_value) + (1.0f - m_alpha) * m_accumulator;
        }

        float getAverage()
        {
            return m_accumulator;
        }

    private:
        float m_alpha, m_accumulator;
    };

    /*************************
     * NUMBER UTILS - FINISH *
     *************************/

}  // namespace Utils

#endif  // UTILS_HEADER_H_
