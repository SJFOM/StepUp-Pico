/**
 * RP2040 FreeRTOS Template
 *
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.4.1
 * @license   MIT
 *
 */
#ifndef MAIN_H
#define MAIN_H

 // FreeRTOS
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
// CXX
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>
// Pico SDK
#include "pico/stdlib.h" // Includes `hardware_gpio.h`
#include "pico/binary_info.h"
// App
#include "../../../Common/utils.h"
// TMC2300 includes
#include "../../Libraries/TMC_API/helpers/CRC.h"
#include "../../Libraries/TMC_API/helpers/Functions.h"
#include "../../Libraries/TMC_API/ic/TMC2300.h"


#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * CONSTANTS
   */
#define RED_LED_PIN 20

   /**
    * PROTOTYPES
    */
  void setup();
  void setup_led();
  void setup_tmc2300();

  void led_on();
  void led_off();
  void led_set(bool state = true);

  void led_task_pico(void* unused_arg);
  void led_task_gpio(void* unused_arg);
  void log_debug(const char* msg);
  void log_device_info(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // MAIN_H