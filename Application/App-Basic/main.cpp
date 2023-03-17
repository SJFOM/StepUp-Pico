/**
 * RP2040 FreeRTOS Template
 *
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.4.1
 * @licence   MIT
 *
 */
#include "include/main.h"

using std::string;
using std::stringstream;
using std::vector;

/*
 * SETUP FUNCTIONS
 */

 /**
  * @brief Hardware setup routine.
  */
void setup() {
    setup_led();
}


/*
 * LED FUNCTIONS
 */

 /**
  * @brief Configure the on-board LED.
  */
void setup_led() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    led_off();
}

/**
 * @brief Turn the on-board LED on.
 */
void led_on() { led_set(); }

/**
 * @brief Turn the on-board LED off.
 */
void led_off() { led_set(false); }

/**
 * @brief Set the on-board LED's state.
 */
void led_set(bool state) { gpio_put(PICO_DEFAULT_LED_PIN, state); }

/**
 * @brief Toggle the on-board LED's state.
 */
void led_toggle()
{
    static bool led_state = false;
    led_state != led_state;
    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
}

/*
 * RUNTIME START
 */
int main() {
    // Enable STDIO
#ifdef DEBUG
    stdio_usb_init();
    // stdio_init_all();
    // Utils::log_device_info();
    sleep_ms(2000);
#endif

    Utils::log_debug("Setting up peripherals...");
    setup();
    Utils::log_debug("OK!");

    // Log app info
    Utils::log_device_info();

    unsigned long c = 0;

    // Blink the on-board LED and print out our counter value
    while (true) {
        led_toggle();
        Utils::log_debug("ticky ticker: ");
        Utils::log_debug(std::to_string(c));
        c++;
        sleep_ms(1000);
    };
}
