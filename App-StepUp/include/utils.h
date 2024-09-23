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
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
// Pico SDK
#include "pico/binary_info.h"
#include "pico/stdlib.h"

// ADC ENOB ~= 8.7 - See Section 4.9.3 of datasheet
#define ADC_ENOB_MASK (0xFF8)  // top 9 MSB's are valid

// ADC homing position allowable threshold values
#define ADC_MIDWAY_VALUE_RAW \
    (1 << 11U)  // 12 bit ADC, take mid-way value (half)

// 12-bit conversion, assume max value == ADC_VREF == 3.3 V
#define ADC_TO_VOLTAGE_CONVERSION_FACTOR (3.3f / (1 << 12))

using std::string;
using std::vector;

/*
 * PROTOTYPES
 */
namespace Utils
{
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
}  // namespace Utils

#endif  // UTILS_HEADER
