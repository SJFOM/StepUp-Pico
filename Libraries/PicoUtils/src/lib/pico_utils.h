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

#ifndef PICO_UTILS_HEADER_H_
#define PICO_UTILS_HEADER_H_

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

// Utils Library
#include <Utils.h>

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

/*
 * PROTOTYPES
 */
namespace PicoUtils
{

    /***************************
     * TIMESTAMP UTILS - START *
     ***************************/
    uint32_t getCurrentTimestampMs();

    uint32_t getElapsedTimeMs(uint32_t start_time_ms);

    /****************************
     * TIMESTAMP UTILS - FINISH *
     ****************************/

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

}  // namespace PicoUtils

#endif  // PICO_UTILS_HEADER_H_
