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
volatile QueueHandle_t queue_joystick_speed = NULL;

// Set a delay time of exactly 500ms
const TickType_t ms_delay = 500 / portTICK_PERIOD_MS;
const TickType_t tmc_job_delay = 200 / portTICK_PERIOD_MS;
const TickType_t joystick_job_delay = 100 / portTICK_PERIOD_MS;

// FROM 1.0.1 Record references to the tasks
TaskHandle_t gpio_task_handle = NULL;
TaskHandle_t pico_task_handle = NULL;
TaskHandle_t tmc_task_handle = NULL;
TaskHandle_t joystick_task_handle = NULL;

// Create class instances of control interfaces
TMCControl tmc_control;
JoystickControl joystick_control;


/*
 * SETUP FUNCTIONS
 */

 /**
  * @brief Hardware setup routine.
  */
void setup() {
    setup_led();
    setup_tmc2300();
    setup_joystick();
}

/**
 * @brief Set the up TMC2300 IC
 *
 * @details Configure UART, power on IC, reset register state to defaults, verify silicon version
 *
 */
void setup_tmc2300()
{
    // If this fails on a call to writing to TMC then it will be blocking!
    if (false == tmc_control.init())
    {
        Utils::log_info("ERROR:TMC failed to initialise!");
    }

    uint8_t tmc_version = tmc_control.getChipID();

    if (tmc_version == TMC2300_VERSION_COMPATIBLE)
    {
        Utils::log_info("TMC2300 silicon version: 0x40");
    }
    else
    {
        Utils::log_error("TMC version: INVALID!");
    }
}

/**
 * @brief Set the up Josytick module
 *
 * @details // TODO: Fill in details
 *
 */
void setup_joystick()
{
    // If this fails on a call to writing to TMC then it will be blocking!
    if (false == joystick_control.init())
    {
        // This will be true if no joystick present OR the josytick is not centered
        Utils::log_info("ERROR:Joystick failed to initialise!");
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
        // Utils::log_info("PICO LED FLASH");
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
                // Utils::log_info("GPIO LED FLASH");
                ;
            }
            gpio_put(RED_LED_PIN, passed_value_buffer == 1 ? 0 : 1);
        }
    }
}

/**
 * @brief Repeatedly flash the Pico's built-in LED.
 */
void tmc_process_job(void* unused_arg) {
    
    ControllerState tmc_state = ControllerState::STATE_IDLE;
    // JoystickState joystick_state = JoystickState::JOYSTICK_STATE_IDLE; 

    int32_t joystick_velocity = 0;
    
    // Store the Pico LED state
    uint8_t pico_led_state = 0;

    // Configure the Pico's on-board LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    unsigned long count = 0;

    bool default_config_sent = false;

    while (true) {
        // Turn Pico LED on an add the LED state
        // to the FreeRTOS xQUEUE
        // Utils::log_info("TMC PROCESS JOB");
        printf("count: %lu\n", count++);
        pico_led_state ^= 1;
        printf("led state: %d\n", pico_led_state);
        
        gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
        xQueueSendToBack(queue, &pico_led_state, 0);
        vTaskDelay(tmc_job_delay);

        tmc_state = tmc_control.processJob(xTaskGetTickCount());

        printf("TMC state: %s\n", ControllerStateString[tmc_state]);

        switch(tmc_state)
        {
            case ControllerState::STATE_IDLE:
            {
                // Only fall here if peripheral not yet initialised
                break;
            }
            case ControllerState::STATE_READY:
            {
                if(!default_config_sent)
                {
                    printf("Set up default config\n");
                    tmc_control.defaultConfiguration();
                    default_config_sent = true;
                }
                // Check for an item in the FreeRTOS xQueue
                if (xQueueReceive(queue_joystick_speed, &joystick_velocity, portMAX_DELAY) == pdPASS) 
                {
                    tmc_control.setCurrent(5,0);
                    tmc_control.move(joystick_velocity);
                }

                break;
            }
            case ControllerState::STATE_NEW_DATA:
            {
                // tmc_control get State
                break;
            }
            case ControllerState::STATE_BUSY:
            default:
                // Wait until the tmc2300 is configured
                break;
        }
    }
}


void joystick_process_job(void *unused_arg)
{
    unsigned long count = 0;
    ControllerState joystick_controller_state = ControllerState::STATE_IDLE;
    struct JoystickData joystick_data = {};

    while (true) {
        joystick_controller_state = joystick_control.processJob(xTaskGetTickCount());
        if(joystick_controller_state == ControllerState::STATE_NEW_DATA)
        {
            joystick_data = joystick_control.getJoystickData();
            int velocity = 0;
            switch(joystick_data.joystick_state)
            {
                case(JoystickState::JOYSTICK_STATE_IDLE):
                    velocity = 0;
                    break;
                case(JoystickState::JOYSTICK_STATE_LOW):
                    velocity = 4000;
                    break;
                case(JoystickState::JOYSTICK_STATE_MID_1):
                    velocity = 10000;
                    break;
                case(JoystickState::JOYSTICK_STATE_MID_2):
                    velocity = 14000;
                    break;
                case(JoystickState::JOYSTICK_STATE_HIGH_1):
                    velocity = 16000;
                    break;
                case(JoystickState::JOYSTICK_STATE_HIGH_2):
                    velocity = 20000;
                    break;
                default:
                    velocity = 0;
            }
            velocity *= (joystick_data.x_stage.x_pos > 0) ? 1 : -1;
            xQueueSendToBack(queue_joystick_speed, &velocity, 0);
        }
        vTaskDelay(joystick_job_delay);
    }
}

/*
 * RUNTIME START
 */
int main() {
    // Enable STDIO
#ifdef DEBUG
    // stdio_usb_init();
    stdio_init_all();
    sleep_ms(2000);
    // Log app info
    Utils::log_device_info();
#endif

    Utils::log_info("Setting up peripherals...");
    setup();
    Utils::log_info("Setting up peripherals - OK!");

    // FROM 1.0.1 Store handles referencing the tasks; get return values
    // NOTE Arg 3 is the stack depth -- in words, not bytes
    BaseType_t gpio_status = xTaskCreate(led_task_gpio, "GPIO_LED_TASK", 128,
        NULL, 1, &gpio_task_handle);
    BaseType_t joystick_status = xTaskCreate(joystick_process_job, "JOYSTICK_JOB_TASK", 512,
        NULL, 2, &joystick_task_handle);
    BaseType_t tmc_status = xTaskCreate(tmc_process_job, "TMC_JOB_TASK", 256,
        NULL, 3, &tmc_task_handle);

    // Set up the event queue
    queue = xQueueCreate(1, sizeof(uint8_t));
    queue_joystick_speed = xQueueCreate(2, sizeof(int32_t));

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (gpio_status == pdPASS && tmc_status == pdPASS && joystick_status == pdPASS) {
        vTaskStartScheduler();
    }

    // We should never get here, but just in case...
    while (true) {
        // NOP
    };
}
