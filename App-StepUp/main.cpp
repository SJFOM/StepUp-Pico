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
volatile QueueHandle_t queue_led_task = NULL;
volatile QueueHandle_t queue_motor_control_data = NULL;
volatile QueueHandle_t queue_notification_task = NULL;

// Set loop delay times (in ms)
const TickType_t joystick_job_delay_ms = 10 / portTICK_PERIOD_MS;
const TickType_t tmc_job_delay_ms = 20 / portTICK_PERIOD_MS;
const TickType_t buzzer_job_delay_ms = 100 / portTICK_PERIOD_MS;
const TickType_t led_job_delay_ms = 100 / portTICK_PERIOD_MS;

// FROM 1.0.1 Record references to the tasks
TaskHandle_t joystick_task_handle = NULL;
TaskHandle_t tmc_task_handle = NULL;
TaskHandle_t buzzer_task_handle = NULL;
TaskHandle_t led_task_handle = NULL;

// Task priorities (higher value = higher priority)
UBaseType_t job_priority_joystick_control = 3U;
UBaseType_t job_priority_tmc_control = 2U;
UBaseType_t job_priority_buzzer_control = 2U;
UBaseType_t job_priority_led_control = 1U;

// Create class instances of control interfaces
TMCControl tmc_control;
JoystickControl joystick_control;
BuzzerControl buzzer_control;
LEDControl led_control;

// FIXME: Remove this prototype function
bool flash_test();

/*
 * SETUP FUNCTIONS
 */

/**
 * @brief Hardware setup routine.
 */
void setup()
{
    assert(flash_test() == true);
    setup_adc();
    setup_led();
    setup_tmc2300();
    setup_boost_converter();
    setup_joystick();
    setup_buzzer();
}

void print_buf(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        printf("%02x", buf[i]);
        if (i % 16 == 15)
            printf("\n");
        else
            printf(" ");
    }
}

bool flash_test()
{
    // 128Mbit / 8 -> 16MByte
    // NOTE: Artificially capping this to 2MB here as there are several .h
    // definitions which say the flash is actually 2MB in size (as per the Pico
    // dev board). This would require changes to openocd also.
    const uint32_t flash_chip_size_mb =
        2 * 1024 * 1024;  // 2048Kb = 2MB = 0x200000
    // const uint32_t flash_chip_size_mb = 256 * 1024;  // 256kb = 0x40000
    const uint32_t flash_target_offset =
        (flash_chip_size_mb - FLASH_SECTOR_SIZE);

    const uint8_t *flash_target_contents =
        (const uint8_t *)(XIP_BASE + flash_target_offset);

    uint8_t random_data[FLASH_PAGE_SIZE];
    for (uint i = 0; i < FLASH_PAGE_SIZE; ++i)
    {
        random_data[i] = (rand() >> 16);
    }

    printf("Generated random data:\n");
    print_buf(random_data, FLASH_PAGE_SIZE);

    // N.B.: Erase is REQUIRED before writing to flash - learned this the hard
    // way!!! Note that a whole number of sectors must be erased at a time.
    printf("\nErasing target region...\n");
    flash_range_erase(flash_target_offset, FLASH_SECTOR_SIZE);
    printf("Done. Read back target region:\n");
    print_buf(flash_target_contents, FLASH_PAGE_SIZE);

    printf("\nProgramming target region...\n");
    flash_range_program(flash_target_offset, random_data, FLASH_PAGE_SIZE);
    printf("Done. Read back target region:\n");
    print_buf(flash_target_contents, FLASH_PAGE_SIZE);

    bool mismatch = false;
    for (uint i = 0; i < FLASH_PAGE_SIZE; ++i)
    {
        if (random_data[i] != flash_target_contents[i])
        {
            mismatch = true;
        }
    }
    if (mismatch)
    {
        printf("Programming failed!\n");
    }
    else
    {
        printf("Programming successful!\n");
    }

    return !mismatch;
}

void setup_adc()
{
    Utils::log_info("ADC setup...");
    adc_init();

    adc_gpio_init(VBAT_MONITOR_ADC_PIN);

    // Give time for the voltage on the boost converter ADC pin to settle
    sleep_ms(100);

    uint16_t battery_voltage_raw =
        Utils::getValidADCResultRaw(VBAT_MONITOR_ADC_CHANNEL);

    float battery_voltage_volts =
        Utils::getValidADCResultVolts(VBAT_MONITOR_ADC_CHANNEL);

    Utils::log_info((string) "Battery voltage - raw value: " +
                    std::to_string(battery_voltage_raw) + " - voltage: " +
                    std::to_string(battery_voltage_volts) + " V");

    float battery_voltage_scaled =
        battery_voltage_volts * VBAT_ADC_SCALING_FACTOR;

    Utils::log_info(
        (string) "Battery voltage - scaled: " +
        std::to_string(battery_voltage_volts * VBAT_ADC_SCALING_FACTOR) + " V");

    // VBat voltage should be greater than 3.3V (ADC: 2048) and less than 4.3
    // volts (ADC: ~2793)
    // TODO: Encode these values in a header file for a VBAT monitoring task
    if (!Utils::isValueWithinBounds(battery_voltage_raw, 2048, 2793))
    {
        Utils::log_error("Battery voltage out of range... FAIL");
    }

    Utils::log_info("ADC setup... OK");
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
    Utils::log_info("TMC2300 setup...");

    bool tmc_setup_success = true;
    // If this fails on a call to writing to TMC then it will be blocking!
    tmc_setup_success &= tmc_control.init();

    if (tmc_control.getChipID() == TMC2300_VERSION_COMPATIBLE)
    {
        Utils::log_info("TMC2300 silicon version: 0x40");
    }
    else
    {
        Utils::log_warn("TMC version: UNSUPPORTED!");
        tmc_setup_success = false;
    }

    if (tmc_setup_success)
    {
        tmc_control.enableFunctionality(true);
        Utils::log_info("TMC2300 setup... OK");
    }
    else
    {
        Utils::log_error("TMC2300 setup... FAIL");
    }
}

void setup_boost_converter()
{
    Utils::log_info("Boost converter setup...");
    // TODO:
    // 1 - enable ADC and pins
    // 2 - enable boost pin
    // 3 - check boost voltage is within target range

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

    Utils::log_info(
        (string) "Boost converter - raw value: " +
        std::to_string(boost_converter_voltage_raw) +
        " - voltage: " + std::to_string(boost_converter_voltage_volts) + " V");

    Utils::log_info((string) "Boost converter - output voltage: " +
                    std::to_string(boost_converter_voltage_volts *
                                   VMOTOR_ADC_SCALING_FACTOR) +
                    " V");

    // VMotor voltage should sit around 1.65V if Vmotor = 10.6V
    if (!Utils::isValueWithinBounds(boost_converter_voltage_raw,
                                    ADC_MIDWAY_VALUE_RAW - 200,
                                    ADC_MIDWAY_VALUE_RAW + 200))
    {
        Utils::log_error("Boost converter setup... FAIL");
    }
    Utils::log_info("Boost converter setup... OK");
}

/**
 * @brief Set the up joystick object
 *
 */
void setup_joystick()
{
    Utils::log_info("Joystick setup...");

    if (joystick_control.init())
    {
        joystick_control.enableFunctionality(true);
    }
    else
    {
        // This will be true if no joystick present OR the josytick is not
        // centered
        Utils::log_error("Joystick setup... FAIL");
    }
    Utils::log_info("Joystick setup... OK");
}

void setup_buzzer()
{
    Utils::log_info("Buzzer setup...");
    if (buzzer_control.init())
    {
        buzzer_control.enableFunctionality(true);
        buzzer_control.setBuzzerFunction(ControllerNotification::NOTIFY_BOOT);
    }
    else
    {
        // FIXME: When will this ever be true?
        Utils::log_error("Buzzer setup... FAIL");
    }
    Utils::log_info("Buzzer setup... OK");
}

void setup_led()
{
    Utils::log_info("LED setup...");
    if (led_control.init())
    {
        led_control.enableFunctionality(true);
        led_control.setLEDFunction(ControllerNotification::NOTIFY_BOOT);
    }
    else
    {
        // FIXME: When will this ever be true?
        Utils::log_error("LED setup... FAIL");
    }
    Utils::log_info("LED setup... OK");
}

/*
 * TASK FUNCTIONS
 */

/**
 * @brief Repeatedly flash the Pico's built-in LED.
 */
[[noreturn]] void tmc_process_job(void *unused_arg)
{
    enum ControllerNotification tmc_notify =
        ControllerNotification::NOTIFY_BOOT;
    ControllerState tmc_state = ControllerState::STATE_IDLE;
    TMCData tmc_data = {};

    // Configure the Pico's on-board LED
    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);

    unsigned long count = 0;

    bool default_config_sent = false;

    while (true)
    {
        vTaskDelay(tmc_job_delay_ms);

        tmc_state = tmc_control.processJob(xTaskGetTickCount());

        // Utils::log_debug("TMC state: " +
        //                  (string)ControllerStateString[tmc_state]);

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
                        Utils::log_info("Button press - stopping motor!");
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
                    // Most likely case is to emit a WARN signal if we enter
                    // this state
                    tmc_notify = ControllerNotification::NOTIFY_WARN;

                    if (tmc_data.diag.open_circuit)
                    {
                        Utils::log_debug("Open circuit detected!");
                    }
                    if (tmc_data.diag.overheating)
                    {
                        // Overheat event warrants a more assertive notification
                        // to the user
                        tmc_notify = ControllerNotification::NOTIFY_ERROR;
                        Utils::log_debug("Overheating!");
                        tmc_control.enableFunctionality(false);
                    }
                    if (tmc_data.diag.short_circuit)
                    {
                        Utils::log_debug("Short circuit detected!");
                    }

                    xQueueSendToBack(queue_notification_task, &tmc_notify, 0);
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

[[noreturn]] void joystick_process_job(void *unused_arg)
{
    unsigned long count = 0;
    enum ControllerNotification joystick_notify =
        ControllerNotification::NOTIFY_BOOT;
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

                if (motor_data.button_press)
                {
                    joystick_notify = ControllerNotification::NOTIFY_INFO;

                    xQueueSendToBack(queue_notification_task,
                                     &joystick_notify,
                                     0);
                }

                // To ensure our new data isn't discarded and successfully
                // makes it into the queue, we should wait a bit longer than
                // the amount of time required to process one tmc_job_delay_ms
                // period to allow the tmc task to complete its latest round
                // of actions.
                xQueueSendToBack(queue_motor_control_data,
                                 &motor_data,
                                 tmc_job_delay_ms + 1);
                break;
            }
            case ControllerState::STATE_BUSY:
            default:
                break;
        }
        vTaskDelay(joystick_job_delay_ms);
    }
}

[[noreturn]] void buzzer_process_job(void *unused_arg)
{
    unsigned long count = 0;
    enum ControllerNotification buzzer_notify =
        ControllerNotification::NOTIFY_BOOT;
    ControllerState buzzer_controller_state = ControllerState::STATE_IDLE;

    while (true)
    {
        buzzer_controller_state =
            buzzer_control.processJob(xTaskGetTickCount());
        // printf("Buzzer queue read, state: %s\n",
        //        ControllerStateString[buzzer_controller_state]);
        switch (buzzer_controller_state)
        {
            case ControllerState::STATE_IDLE:
            {
                break;
            }
            case ControllerState::STATE_READY:
            {
                if (xQueueReceive(queue_notification_task,
                                  &buzzer_notify,
                                  portMAX_DELAY) == pdPASS)
                {
                    buzzer_control.setBuzzerFunction(buzzer_notify);
                }
                break;
            }
            case ControllerState::STATE_NEW_DATA:
            case ControllerState::STATE_BUSY:
            default:
                break;
        }
        vTaskDelay(buzzer_job_delay_ms);
    }
}

[[noreturn]] void led_process_job(void *unused_arg)
{
    unsigned long count = 0;
    enum ControllerNotification led_notify =
        ControllerNotification::NOTIFY_BOOT;
    ControllerState led_controller_state = ControllerState::STATE_IDLE;

    while (true)
    {
        led_controller_state = led_control.processJob(xTaskGetTickCount());

        // printf("LED queue read, state: %s\n",
        //        ControllerStateString[led_controller_state]);

        switch (led_controller_state)
        {
            case ControllerState::STATE_IDLE:
            {
                break;
            }
            case ControllerState::STATE_READY:
            {
                if (xQueueReceive(queue_notification_task,
                                  &led_notify,
                                  portMAX_DELAY) == pdPASS)
                {
                    led_control.setLEDFunction(led_notify);
                }
                break;
            }
            case ControllerState::STATE_NEW_DATA:
            case ControllerState::STATE_BUSY:
            default:
                break;
        }
        vTaskDelay(led_job_delay_ms);
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
    Utils::log_info("Setting up peripherals... OK!");

    // FROM 1.0.1 Store handles referencing the tasks; get return values
    // NOTE Arg 3 is the stack depth -- in words, not bytes
    BaseType_t led_status = xTaskCreate(led_process_job,
                                        "GPIO_LED_TASK",
                                        128,
                                        NULL,
                                        job_priority_led_control,
                                        &led_task_handle);
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

    BaseType_t buzzer_status = xTaskCreate(buzzer_process_job,
                                           "BUZZER_JOB_TASK",
                                           256,
                                           NULL,
                                           job_priority_buzzer_control,
                                           &buzzer_task_handle);

    // Set up the event queue
    queue_led_task = xQueueCreate(1, sizeof(uint8_t));
    queue_motor_control_data = xQueueCreate(2, sizeof(struct MotorControlData));
    queue_notification_task =
        xQueueCreate(1, sizeof(enum ControllerNotification));

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (led_status == pdPASS && tmc_status == pdPASS &&
        joystick_status == pdPASS && buzzer_status == pdPASS)
    {
        vTaskStartScheduler();
    }

    // We should never get here, but just in case...
    while (true)
    {
        // NOP
    }
}
