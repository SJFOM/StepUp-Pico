/**
 * RP2040 FreeRTOS Template - App #2
 * General utility functions
 *
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
