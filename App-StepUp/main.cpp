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
volatile QueueHandle_t queue_motor_control_data = NULL;

// Motor control define(s)
#define VELOCITY_DELTA_VALUE (500U)

struct MotorControlData
{
    int32_t velocity_delta;
    int8_t direction;
    bool button_press;
};

// Set a delay time of exactly 500ms
const TickType_t ms_delay = 500 / portTICK_PERIOD_MS;
const TickType_t tmc_job_delay = 20 / portTICK_PERIOD_MS;
const TickType_t joystick_job_delay = 10 / portTICK_PERIOD_MS;

// FROM 1.0.1 Record references to the tasks
TaskHandle_t gpio_task_handle = NULL;
TaskHandle_t pico_task_handle = NULL;
TaskHandle_t tmc_task_handle = NULL;
TaskHandle_t joystick_task_handle = NULL;

// Task priorities (higher value = higher priority)
UBaseType_t job_priority_led_control = 1U;
UBaseType_t job_priority_tmc_control = 2U;
UBaseType_t job_priority_joystick_control = 3U;

// Create class instances of control interfaces
TMCControl tmc_control;
JoystickControl joystick_control;
BuzzerControl buzzer_control;

/*
 * SETUP FUNCTIONS
 */

/**
 * @brief Hardware setup routine.
 */
void setup()
{
    Utils::log_info("ADC setup");
    adc_init();
    Utils::log_info("LED setup");
    setup_led();
    Utils::log_info("TMC2300 setup");
    setup_tmc2300();
    Utils::log_info("Boost converter setup");
    setup_boost_converter();
    Utils::log_info("Joystick setup");
    setup_joystick();
    Utils::log_info("Buzzer setup");
    setup_buzzer();
    Utils::log_info("Setup complete!");
}

/**
 * @brief Set the up TMC2300 IC
 *
 * @details Configure UART, power on IC, reset register state to defaults,
 * verify silicon version
 *
 */
void setup_tmc2300()
{
    // If this fails on a call to writing to TMC then it will be blocking!
    if (tmc_control.init())
    {
        uint8_t tmc_version = tmc_control.getChipID();

        if (tmc_version == TMC2300_VERSION_COMPATIBLE)
        {
            Utils::log_info("TMC2300 silicon version: 0x40");
        }
        else
        {
            Utils::log_error("TMC version: INVALID!");
        }

        tmc_control.enableFunctionality(true);
    }
    else
    {
        Utils::log_error("TMC failed to initialise!");
    }
}

void setup_boost_converter()
{
    // TODO:
    // 1 - enable ADC and pins
    // 2 - enable boost pin
    // 3 - check boost voltage is within target range

    if (!Utils::isADCInitialised())
    {
        adc_init();
    }

    adc_gpio_init(VMOTOR_MONITOR_ADC_PIN);

    // Enable boost converter pin control
    gpio_init(TMC_PIN_BOOST_EN);
    gpio_set_dir(TMC_PIN_BOOST_EN, GPIO_OUT);

    // Enable boost converter
    gpio_put(TMC_PIN_BOOST_EN, 1);

    // Give time for the voltage on the boost converter ADC pin to settle
    sleep_ms(100);

    uint16_t boost_converter_voltage_raw =
        Utils::getValidADCResultRaw(VMOTOR_MONITOR_ADC_CHANNEL);

    float boost_converter_voltage_volts =
        Utils::getValidADCResultVolts(VMOTOR_MONITOR_ADC_CHANNEL);

    printf("boost converter voltage - raw: %d\n", boost_converter_voltage_raw);
    printf("boost converter voltage - volts: %.3f\n",
           boost_converter_voltage_volts);

    // VMotor voltage should sit around 1.65V if Vmotor = 10.6V
    if (!Utils::isValueWithinBounds(boost_converter_voltage_raw,
                                    ADC_MIDWAY_VALUE_RAW - 200,
                                    ADC_MIDWAY_VALUE_RAW + 200))
    {
        Utils::log_error("Boost converter voltage abnormal!");
    }
}

/**
 * @brief Set the up joystick object
 *
 */
void setup_joystick()
{
    if (joystick_control.init())
    {
        joystick_control.enableFunctionality(true);
    }
    else
    {
        // This will be true if no joystick present OR the josytick is not
        // centered
        Utils::log_error("Joystick failed to initialise!");
    }
}

void setup_buzzer()
{
    if (buzzer_control.init())
    {
        buzzer_control.enableFunctionality(true);
    }
    else
    {
        // FIXME: When will this ever be true?
        Utils::log_error("Buzzer failed to initialise!");
    }
}

/*
 * LED FUNCTIONS
 */

/**
 * @brief Configure the on-board LED.
 */
void setup_led()
{
    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);
    led_off();
}

/**
 * @brief Turn the on-board LED on.
 */
void led_on()
{
    led_set();
}

/**
 * @brief Turn the on-board LED off.
 */
void led_off()
{
    led_set(false);
}

/**
 * @brief Set the on-board LED's state.
 */
void led_set(bool state)
{
    gpio_put(LED_PIN_RED, state);
}

/*
 * TASK FUNCTIONS
 */

/**
 * @brief Repeatedly flash the Pico's built-in LED.
 */
void led_task_pico(void *unused_arg)
{
    // Store the Pico LED state
    uint8_t pico_led_state = 0;

    // Configure the Pico's on-board LED
    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);

    while (true)
    {
        // Turn Pico LED on an add the LED state
        // to the FreeRTOS xQUEUE
        // Utils::log_info("PICO LED FLASH");
        pico_led_state = 1;
        gpio_put(LED_PIN_RED, pico_led_state);
        xQueueSendToBack(queue, &pico_led_state, 0);
        vTaskDelay(ms_delay);

        // Turn Pico LED off an add the LED state
        // to the FreeRTOS xQUEUE
        pico_led_state = 0;
        gpio_put(LED_PIN_RED, pico_led_state);
        xQueueSendToBack(queue, &pico_led_state, 0);
        vTaskDelay(ms_delay);
    }
}

/**
 * @brief Repeatedly flash an LED connected to GPIO pin 20
 *        based on the value passed via the inter-task queue.
 */
void led_task_gpio(void *unused_arg)
{
    // This variable will take a copy of the value
    // added to the FreeRTOS xQueue
    uint8_t passed_value_buffer = 0;

    // Configure the GPIO LED
    gpio_init(LED_PIN_BLUE);
    gpio_set_dir(LED_PIN_BLUE, GPIO_OUT);

    while (true)
    {
        // Check for an item in the FreeRTOS xQueue
        if (xQueueReceive(queue, &passed_value_buffer, portMAX_DELAY) == pdPASS)
        {
            // Received a value so flash the GPIO LED accordingly
            // (NOT the sent value)
            gpio_put(LED_PIN_BLUE, passed_value_buffer == 1 ? 0 : 1);
        }
    }
}

/**
 * @brief Repeatedly flash the Pico's built-in LED.
 */
void tmc_process_job(void *unused_arg)
{
    ControllerState tmc_state = ControllerState::STATE_IDLE;
    TMCData tmc_data = {};

    // Store the Pico LED state
    uint8_t pico_led_state = 0;

    // Configure the Pico's on-board LED
    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);

    unsigned long count = 0;

    bool default_config_sent = false;

    while (true)
    {
        // Turn Pico LED on an add the LED state
        // to the FreeRTOS xQUEUE
        // Utils::log_info("TMC PROCESS JOB");
        // printf("count: %lu\n", count++);
        pico_led_state ^= 1;

        gpio_put(LED_PIN_RED, pico_led_state);
        xQueueSendToBack(queue, &pico_led_state, 0);
        vTaskDelay(tmc_job_delay);

        tmc_state = tmc_control.processJob(xTaskGetTickCount());

        // printf("TMC state: %s\n", ControllerStateString[tmc_state]);

        switch (tmc_state)
        {
            case ControllerState::STATE_IDLE:
            {
                // Only fall here if peripheral not yet initialised
                break;
            }
            case ControllerState::STATE_READY:
            {
                static MotorControlData motor_data = {};
                if (!default_config_sent)
                {
                    Utils::log_info("Configure TMC2300 default values...");
                    tmc_control.defaultConfiguration();
                    default_config_sent = true;
                    Utils::log_info("Configure TMC2300 default values - OK!");
                }
                // Check for an item in the FreeRTOS xQueue
                if (xQueueReceive(queue_motor_control_data, &motor_data, 0) ==
                    pdPASS)
                {
                    // printf("Velocity diff: %d\n",
                    // motor_data.velocity_delta); printf("Direction: %d\n",
                    // motor_data.direction);
                    if (motor_data.button_press)
                    {
                        printf("Button press - stopping motor!\n");
                        // A button press event should instantly stop the
                        // motor
                        tmc_control.resetMovementDynamics();
                    }
                    else
                    {
                        tmc_control.updateMovementDynamics(
                            motor_data.velocity_delta,
                            motor_data.direction);
                    }
                }
                break;
            }
            case ControllerState::STATE_NEW_DATA:
            {
                // tmc_control get State
                tmc_data = tmc_control.getTMCData();
                if (tmc_data.diag.normal_operation)
                {
                    tmc_control.enableFunctionality(true);
                }
                else
                {
                    // TODO: Deal with the issue at hand and report to user
                    if (tmc_data.diag.open_circuit)
                    {
                        Utils::log_debug("Open circuit");
                    }
                    if (tmc_data.diag.overheating)
                    {
                        Utils::log_debug("Overheating");
                        tmc_control.enableFunctionality(false);
                    }
                    if (tmc_data.diag.short_circuit)
                    {
                        Utils::log_debug("Short circuit");
                    }
                    if (tmc_data.diag.stall_detected)
                    {
                        Utils::log_debug("Stall detected");
                    }
                }
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
    JoystickData joystick_data = {};

    while (true)
    {
        joystick_controller_state =
            joystick_control.processJob(xTaskGetTickCount());
        switch (joystick_controller_state)
        {
            case ControllerState::STATE_IDLE:
            {
                break;
            }
            case ControllerState::STATE_NEW_DATA:
            {
                MotorControlData motor_data = {};
                joystick_data = joystick_control.getJoystickData();

                // Assign motor direction to the state of the joystick
                // (either NEGATIVE:-1, IDLE:0, POSITIVE:+1)
                motor_data.direction = joystick_data.state_x;

                // Assign velocity_delta to the state of the joystick
                // (either NEGATIVE:-1, IDLE:0, POSITIVE:+1) multiplied by
                // the change in velocity we should incur.
                motor_data.velocity_delta =
                    joystick_data.state_y * VELOCITY_DELTA_VALUE;

                // This ensures that, no matter which direction we face, the
                // joystick will "speed up" or "slow down" consistent with
                // the direction of rotation of the motor shaft.
                motor_data.velocity_delta *= motor_data.direction;

                motor_data.button_press = joystick_data.button_is_pressed;

                // To ensure our new data isn't discarded and successfully
                // makes it into the queue, we should wait a bit longer than
                // the amount of time required to process one tmc_job_delay
                // period to allow the tmc task to complete its latest round
                // of actions.
                xQueueSendToBack(queue_motor_control_data,
                                 &motor_data,
                                 tmc_job_delay + 1);
                break;
            }
            case ControllerState::STATE_BUSY:
            default:
                break;
        }
        vTaskDelay(joystick_job_delay);
    }
}

/*
 * RUNTIME START
 */
int main()
{
    // Enable either STDIO (UART) or USB (don't enable both)
    stdio_init_all();
    // stdio_usb_init();
    sleep_ms(2000);
    // Log app info
    Utils::log_device_info();

    Utils::log_info("Setting up peripherals...");
    setup();
    Utils::log_info("Setting up peripherals - OK!");

    // FROM 1.0.1 Store handles referencing the tasks; get return values
    // NOTE Arg 3 is the stack depth -- in words, not bytes
    BaseType_t gpio_status = xTaskCreate(led_task_gpio,
                                         "GPIO_LED_TASK",
                                         128,
                                         NULL,
                                         job_priority_led_control,
                                         &gpio_task_handle);
    BaseType_t tmc_status = xTaskCreate(tmc_process_job,
                                        "TMC_JOB_TASK",
                                        256,
                                        NULL,
                                        job_priority_tmc_control,
                                        &tmc_task_handle);
    BaseType_t joystick_status = xTaskCreate(joystick_process_job,
                                             "JOYSTICK_JOB_TASK",
                                             512,
                                             NULL,
                                             job_priority_joystick_control,
                                             &joystick_task_handle);

    // Set up the event queue
    queue = xQueueCreate(4, sizeof(uint8_t));
    queue_motor_control_data = xQueueCreate(2, sizeof(struct MotorControlData));

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (gpio_status == pdPASS && tmc_status == pdPASS &&
        joystick_status == pdPASS)
    {
        vTaskStartScheduler();
    }

    // We should never get here, but just in case...
    while (true)
    {
        // NOP
    }
}
