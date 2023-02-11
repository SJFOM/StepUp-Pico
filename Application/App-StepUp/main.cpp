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
 * GLOBALS
 */
 // This is the inter-task queue
volatile QueueHandle_t queue = NULL;

// Set a delay time of exactly 500ms
const TickType_t ms_delay = 500 / portTICK_PERIOD_MS;

// FROM 1.0.1 Record references to the tasks
TaskHandle_t gpio_task_handle = NULL;
TaskHandle_t pico_task_handle = NULL;

/*
 * SETUP FUNCTIONS
 */

 /**
  * @brief Umbrella hardware setup routine.
  */
void setup() {
    setup_led();
    // setup_tmc2300();
}


#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

/*
 * TMC2300 INIT FUNCTION
 */
void setup_tmc2300() {
    // Debug console

    //TODO: 
    // - Initialise UART at up to 460800 baud
    // - Read some info from the TMC2300 to verify connection

    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART

    // Send out a character without any conversions
    uart_putc_raw(UART_ID, 'A');

    // Send out a character but do CR/LF conversions
    uart_putc(UART_ID, 'B');

    // Send out a string, with CR/LF conversions
    uart_puts(UART_ID, " Hello, UART!\n");

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

/*
 * TASK FUNCTIONS
 */

 /**
  * @brief Repeatedly flash the Pico's built-in LED.
  */
void led_task_pico(void* unused_arg) {
    // Store the Pico LED state
    uint8_t pico_led_state = 0;

    // Configure the Pico's on-board LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (true) {
        // Turn Pico LED on an add the LED state
        // to the FreeRTOS xQUEUE
        Utils::log_debug("PICO LED FLASH");
        pico_led_state = 1;
        gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
        xQueueSendToBack(queue, &pico_led_state, 0);
        vTaskDelay(ms_delay);

        // Turn Pico LED off an add the LED state
        // to the FreeRTOS xQUEUE
        pico_led_state = 0;
        gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
        xQueueSendToBack(queue, &pico_led_state, 0);
        vTaskDelay(ms_delay);
    }
}

/**
 * @brief Repeatedly flash an LED connected to GPIO pin 20
 *        based on the value passed via the inter-task queue.
 */
void led_task_gpio(void* unused_arg) {
    // This variable will take a copy of the value
    // added to the FreeRTOS xQueue
    uint8_t passed_value_buffer = 0;

    // Configure the GPIO LED
    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);

    while (true) {
        // Check for an item in the FreeRTOS xQueue
        if (xQueueReceive(queue, &passed_value_buffer, portMAX_DELAY) == pdPASS) {
            // Received a value so flash the GPIO LED accordingly
            // (NOT the sent value)
            if (passed_value_buffer)
            {
                Utils::log_debug("GPIO LED FLASH");
            }
            gpio_put(RED_LED_PIN, passed_value_buffer == 1 ? 0 : 1);
        }
    }
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

    setup();

    // Set up two tasks
    // FROM 1.0.1 Store handles referencing the tasks; get return values
    // NOTE Arg 3 is the stack depth -- in words, not bytes
    BaseType_t pico_status = xTaskCreate(led_task_pico, "PICO_LED_TASK", 128,
        NULL, 1, &pico_task_handle);
    BaseType_t gpio_status = xTaskCreate(led_task_gpio, "GPIO_LED_TASK", 128,
        NULL, 1, &gpio_task_handle);

    // Set up the event queue
    queue = xQueueCreate(4, sizeof(uint8_t));

    // Log app info
    Utils::log_device_info();

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (pico_status == pdPASS || gpio_status == pdPASS) {
        vTaskStartScheduler();
    }

    // We should never get here, but just in case...
    while (true) {
        // NOP
    };
}
