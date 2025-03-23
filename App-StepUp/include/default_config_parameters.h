/**
 * @file default_config_parameters.h
 * @author Sam (@SJFOM)
 * @brief Single source of truth definitions of common default values
 * @version 0.1
 * @date 2025-03-23
 *
 * @copyright Copyright (c) 2024
 * @license   MIT
 */
#ifndef DEFAULT_CONFIG_PARAMETERS_H_
#define DEFAULT_CONFIG_PARAMETERS_H_

#include <stdint.h>

constexpr static uint32_t s_c_pin_debounce_default_time_in_ms = (50U);

constexpr static uint32_t s_c_velocity_steps_per_second_delta_default_value =
    (500U);

#endif  // DEFAULT_CONFIG_PARAMETERS_H_
