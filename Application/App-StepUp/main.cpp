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
const TickType_t tmc_job_delay = 100 / portTICK_PERIOD_MS;

// FROM 1.0.1 Record references to the tasks
TaskHandle_t gpio_task_handle = NULL;
TaskHandle_t pico_task_handle = NULL;
TaskHandle_t tmc_task_handle = NULL;

// Create class instances of control interfaces
TMCControl tmc_control;

/*
 * SETUP FUNCTIONS
 */

 /**
  * @brief Hardware setup routine.
  */
void setup() {
    setup_led();

    // If this fails on a call to writing to TMC then it will be blocking!
    if (false == tmc_control.init())
    {
        Utils::log_debug("ERROR:TMC failed to initialise!");
    }
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
        // Utils::log_debug("PICO LED FLASH");
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
                // Utils::log_debug("GPIO LED FLASH");
            }
            gpio_put(RED_LED_PIN, passed_value_buffer == 1 ? 0 : 1);
        }
    }
}

/**
 * @brief Repeatedly flash the Pico's built-in LED.
 */
void tmc_process_job(void* unused_arg) {
    // Store the Pico LED state
    uint8_t pico_led_state = 0;

    // Configure the Pico's on-board LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (true) {
        // Turn Pico LED on an add the LED state
        // to the FreeRTOS xQUEUE
        Utils::log_debug("TMC PROCESS JOB");
        pico_led_state = 1;
        tmc_control.processJob(xTaskGetTickCount());
        gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
        xQueueSendToBack(queue, &pico_led_state, 0);
        vTaskDelay(tmc_job_delay);

        // Turn Pico LED off an add the LED state
        // to the FreeRTOS xQUEUE
        pico_led_state = 0;
        gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
        xQueueSendToBack(queue, &pico_led_state, 0);
        vTaskDelay(tmc_job_delay);
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

    Utils::log_debug("Setting up peripherals...");
    setup();
    Utils::log_debug("OK!");

    // Set up two tasks
    // FROM 1.0.1 Store handles referencing the tasks; get return values
    // NOTE Arg 3 is the stack depth -- in words, not bytes
    // BaseType_t pico_status = xTaskCreate(led_task_pico, "PICO_LED_TASK", 128,
        // NULL, 1, &pico_task_handle);
    // BaseType_t gpio_status = xTaskCreate(led_task_gpio, "GPIO_LED_TASK", 128,
    //     NULL, 1, &gpio_task_handle);
    // BaseType_t tmc_status = xTaskCreate(tmc_process_job, "TMC_JOB_TASK", 128,
    //     NULL, 1, &tmc_task_handle);

    // Set up the event queue
    // queue = xQueueCreate(4, sizeof(uint8_t));

    // Log app info
    Utils::log_device_info();

    // Configure the Pico's on-board LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    unsigned long c = 0;
    uint8_t pico_led_state = 0;
    while (true)
    {
        pico_led_state = !pico_led_state;
        gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
        Utils::log_debug("ticky ticker: ");
        Utils::log_debug(std::to_string(c));
        tmc_control.processJob(c);
        if (c == 20)
        {
            tmc_control.enableDriver(true);
        }
        if (c == 40)
        {
            (void)tmc_control.getChipID();
        }
        if (c == 60)
        {
            (void)tmc_control.testFunction();
        }
        sleep_ms(400);
        c++;
    }

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    // if (gpio_status == pdPASS && tmc_status == pdPASS) {
    //     vTaskStartScheduler();
    // }

    // We should never get here, but just in case...
    while (true) {
        // NOP
    };
}
