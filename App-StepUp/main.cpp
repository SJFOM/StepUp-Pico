/**
 * StepUp! main file
 *
 * @copyright 2025, Sam O'Mahony (@SJFOM)
 * @version   0.1
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
volatile QueueHandle_t queue_power_control_data = NULL;
volatile QueueHandle_t queue_motor_control_data = NULL;
volatile QueueHandle_t queue_led_notification_task = NULL;
volatile QueueHandle_t queue_buzzer_notification_task = NULL;
volatile QueueHandle_t queue_led_colour_data = NULL;

// Set loop delay times (in ms)
const TickType_t power_control_job_delay_ms = 1000 / portTICK_PERIOD_MS;
const TickType_t joystick_job_delay_ms = 10 / portTICK_PERIOD_MS;
const TickType_t tmc_job_delay_ms = 20 / portTICK_PERIOD_MS;
const TickType_t led_job_delay_ms = 80 / portTICK_PERIOD_MS;
const TickType_t buzzer_job_delay_ms = 100 / portTICK_PERIOD_MS;
const TickType_t voltage_monitoring_job_delay = 1000 / portTICK_PERIOD_MS;
const TickType_t watchdog_job_delay_ms =
    CX_WATCHDOG_CALLBACK_MS / portTICK_PERIOD_MS;

// FROM 1.0.1 Record references to the tasks
TaskHandle_t power_control_task_handle = NULL;
TaskHandle_t joystick_task_handle = NULL;
TaskHandle_t tmc_task_handle = NULL;
TaskHandle_t led_task_handle = NULL;
TaskHandle_t buzzer_task_handle = NULL;
TaskHandle_t voltage_monitoring_task_handle = NULL;

// Task priorities (higher value = higher priority)
UBaseType_t job_priority_power_control = 3U;
UBaseType_t job_priority_joystick_control = 2U;
UBaseType_t job_priority_tmc_control = 2U;
UBaseType_t job_priority_buzzer_control = 1U;
UBaseType_t job_priority_led_control = 1U;
UBaseType_t job_priority_voltage_monitoring = 1U;

// Create class instances of control interfaces
TMCControl tmc_control(CX_R_SENSE, CX_COOLSTEP_ENABLED);
JoystickControl joystick_control;
BuzzerControl buzzer_control(BUZZER_PIN);
LEDControl led_control(LED_PIN_RED, LED_PIN_GREEN, LED_PIN_BLUE);
PowerControl power_control(CX_POWER_BUTTON_OFF_HOLD_TIMEOUT_MS,
                           CX_POWER_DOWN_INACTIVE_TIMEOUT_MS);

// Create voltage monitoring class instances
VoltageMonitoring battery_voltage_monitoring(
    "Battery",
    VBAT_MONITOR_ADC_PIN,
    VBAT_MONITOR_ADC_CHANNEL,
    VBAT_ADC_SCALING_FACTOR,
    CX_BATTERY_VOLTAGE_THRESHOLD_LOW,
    CX_BATTERY_VOLTAGE_THRESHOLD_HIGH,
    CX_BATTERY_VOLTAGE_DELTA_THRESHOLD,
    VoltageBoundsSensitivityState::VOLTAGE_STATE_SENSITIVITY_MEDIUM);

VoltageMonitoring motor_voltage_monitoring(
    "Motor",
    VMOTOR_MONITOR_ADC_PIN,
    VMOTOR_MONITOR_ADC_CHANNEL,
    VMOTOR_ADC_SCALING_FACTOR,
    CX_MOTOR_IDLE_VOLTAGE_THRESHOLD_LOW,
    CX_MOTOR_IDLE_VOLTAGE_THRESHOLD_HIGH,
    CX_BATTERY_VOLTAGE_DELTA_THRESHOLD,
    VoltageBoundsSensitivityState::VOLTAGE_STATE_SENSITIVITY_LOW);

/*
 * SETUP FUNCTIONS
 */

/**
 * @brief Hardware setup routine.
 */
void setup()
{
    setup_watchdog();
    setup_power_control();
    setup_buzzer();
    setup_led();
    setup_tmc2300();
    setup_boost_converter();
    setup_voltage_monitoring();
    setup_joystick();
}

void setup_watchdog()
{
    LOG_INFO("Watchdog setup...");
    if (watchdog_caused_reboot())
    {
        LOG_INFO("Watchdog caused reboot");
    }
    // Enable the watchdog timer
    watchdog_enable(CX_WATCHDOG_TIMEOUT_MS, 1 /*pause_on_debug*/);

    // Set the watchdog timer to reset the system if it is not fed within the
    // specified timeout period
    watchdog_update();

    LOG_INFO("Watchdog setup... OK");
}

void setup_power_control()
{
    LOG_INFO("Power control setup...");
    if (power_control.init())
    {
        LOG_INFO("Power control setup... OK");
    }
    else
    {
        LOG_ERROR("Power control setup... FAIL");
    }

    if (power_control.isUSBInserted())
    {
        LOG_INFO("USB cable detected");
    }
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
    LOG_INFO("TMC2300 setup...");

    bool tmc_setup_success = true;
    // If this fails on a call to writing to TMC then it will be blocking!
    tmc_setup_success &= tmc_control.init();

    if (tmc_control.getChipID() == TMC2300_VERSION_COMPATIBLE)
    {
        LOG_INFO("TMC2300 silicon version: 0x40");
    }
    else
    {
        LOG_WARN("TMC version: UNSUPPORTED!");
        tmc_setup_success = false;
    }

    if (tmc_setup_success)
    {
        LOG_INFO("TMC2300 setup... OK");
    }
    else
    {
        LOG_ERROR("TMC2300 setup... FAIL");
    }
}

void setup_boost_converter()
{
    LOG_INFO("Boost converter setup...");

    // Enable boost converter pin control
    gpio_init(TMC_PIN_BOOST_EN);
    gpio_set_dir(TMC_PIN_BOOST_EN, GPIO_OUT);

    // Enable boost converter
    gpio_put(TMC_PIN_BOOST_EN, 1);

    LOG_INFO("Boost converter setup... OK");
}

/**
 * @brief Set the up joystick object
 *
 */
void setup_joystick()
{
    LOG_INFO("Joystick setup...");

    if (joystick_control.init())
    {
        joystick_control.enableFunctionality(true);
    }
    else
    {
        // This will be true if no joystick present OR the josytick is not
        // centered
        LOG_ERROR("Joystick setup... FAIL");
    }
    LOG_INFO("Joystick setup... OK");
}

void setup_buzzer()
{
    LOG_INFO("Buzzer setup...");
    if (buzzer_control.init())
    {
        buzzer_control.enableFunctionality(true);
        buzzer_control.setBuzzerFunction(ControllerNotification::NOTIFY_BOOT);
    }
    else
    {
        // FIXME: When will this ever be true?
        LOG_ERROR("Buzzer setup... FAIL");
    }
    LOG_INFO("Buzzer setup... OK");
}

void setup_led()
{
    LOG_INFO("LED setup...");
    if (led_control.init())
    {
        led_control.enableFunctionality(true);
        led_control.setLEDFunction(ControllerNotification::NOTIFY_BOOT);
    }
    else
    {
        // FIXME: When will this ever be true?
        LOG_ERROR("LED setup... FAIL");
    }
    LOG_INFO("LED setup... OK");
}

void setup_voltage_monitoring()
{
    LOG_INFO("Voltage monitoring setup...");
    if (battery_voltage_monitoring.init())
    {
        battery_voltage_monitoring.enableFunctionality(true);
        LOG_INFO("Battery voltage monitoring setup... OK");
    }
    else
    {
        LOG_ERROR("Battery voltage monitoring setup... FAIL");
    }

    // NOTE: This may fail if we do not initialise the boost converter before
    // now
    if (motor_voltage_monitoring.init())
    {
        motor_voltage_monitoring.enableFunctionality(true);

        // Now we know the motor voltage is in spec, we can broaden the expected
        // motor voltage range to accommodate the back-emf generated by a
        // spinning motor (10% bounds)
        motor_voltage_monitoring.setVoltageThresholds(
            CX_MOTOR_ACTIVE_VOLTAGE_THRESHOLD_LOW,
            CX_MOTOR_ACTIVE_VOLTAGE_THRESHOLD_HIGH);
        LOG_INFO("Boost/Motor voltage monitoring setup... OK");
    }
    else
    {
        LOG_ERROR("Boost/Motor voltage monitoring setup... FAIL");
    }

    LOG_INFO("Voltage monitoring setup...OK");
}

/*
 * TASK FUNCTIONS
 */

/**
 * @brief Process tasks on the TMC2300 motor controller IC
 */
[[noreturn]] void tmc_process_job(void *unused_arg)
{
    enum ControllerNotification tmc_notify =
        ControllerNotification::NOTIFY_BOOT;
    ControllerState tmc_state = ControllerState::STATE_IDLE;
    TMCData tmc_data = {};

    unsigned long count = 0;

    bool default_config_sent = false;

    while (true)
    {
        vTaskDelay(tmc_job_delay_ms);

        tmc_state = tmc_control.processJob(xTaskGetTickCount());

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
                    LOG_INFO("Configure TMC2300 default values...");
                    tmc_control.defaultConfiguration();
                    default_config_sent = true;
                    LOG_INFO("Configure TMC2300 default values - OK!");
                }
                // Check for an item in the FreeRTOS xQueue
                if (xQueueReceive(queue_motor_control_data, &motor_data, 0) ==
                    pdPASS)
                {
                    if (motor_data.button_press)
                    {
                        LOG_INFO("Button press - stopping motor!");
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
                    LOG_DEBUG("Normal operation\n");
                }
                else
                {
                    // Most likely case is to emit a WARN signal if we enter
                    // this state
                    tmc_notify = ControllerNotification::NOTIFY_WARN;
                    bool trigger_led = true;
                    bool trigger_buzzer = true;

                    if (tmc_data.diag.stall_detected)
                    {
                        LOG_DEBUG(
                            "Stall detected! Automatically reducing speed");
                        // Don't trigger a typical info or warn message as we
                        // don't need to alert to the user that a max rpm has
                        // been reached as this will likely happen very often
                        trigger_led = false;
                        trigger_buzzer = false;

                        tmc_notify = ControllerNotification::NOTIFY_DATA;

                        enum LEDColourNames led_colour =
                            LEDColourNames::LED_COLOUR_MAGENTA;
                        xQueueSendToBack(queue_led_colour_data, &led_colour, 0);
                    }
                    if (tmc_data.diag.open_circuit ||
                        tmc_data.open_circuit_detected)
                    {
                        // Open circuit detected, trigger a notification
                        tmc_notify = ControllerNotification::NOTIFY_ERROR;
                        LOG_DEBUG("Open circuit detected!");
                        tmc_control.enableFunctionality(false);
                    }
                    if (tmc_data.diag.overheating)
                    {
                        // Overheat event warrants a more assertive notification
                        // to the user
                        tmc_notify = ControllerNotification::NOTIFY_ERROR;
                        LOG_DEBUG("Overheating!");
                        tmc_control.enableFunctionality(false);
                    }
                    if (tmc_data.diag.short_circuit)
                    {
                        LOG_DEBUG("Short circuit detected!");
                    }

                    if (trigger_led)
                    {
                        xQueueSendToBack(queue_led_notification_task,
                                         &tmc_notify,
                                         0);
                    }
                    if (trigger_buzzer)
                    {
                        xQueueSendToBack(queue_buzzer_notification_task,
                                         &tmc_notify,
                                         0);
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
                    joystick_data.state_y *
                    VELOCITY_RAMP_INCREMENT_STEPS_PER_SECOND;

                // This ensures that, no matter which direction we face, the
                // joystick will "speed up" or "slow down" consistent with
                // the direction of rotation of the motor shaft.
                motor_data.velocity_delta *= motor_data.direction;

                motor_data.button_press = joystick_data.button_is_pressed;

                if (motor_data.button_press)
                {
                    joystick_notify = ControllerNotification::NOTIFY_INFO;

                    xQueueSendToBack(queue_led_notification_task,
                                     &joystick_notify,
                                     0);

                    xQueueSendToBack(queue_buzzer_notification_task,
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
        switch (buzzer_controller_state)
        {
            case ControllerState::STATE_IDLE:
            {
                break;
            }
            case ControllerState::STATE_READY:
            {
                if (xQueueReceive(queue_buzzer_notification_task,
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
    enum ControllerNotification led_notify =
        ControllerNotification::NOTIFY_BOOT;
    ControllerState led_controller_state = ControllerState::STATE_IDLE;

    enum LEDColourNames led_colour = LEDColourNames::LED_COLOUR_OFF;

    while (true)
    {
        led_controller_state = led_control.processJob(xTaskGetTickCount());

        switch (led_controller_state)
        {
            case ControllerState::STATE_IDLE:
            {
                break;
            }
            case ControllerState::STATE_READY:
            {
                if (xQueueReceive(queue_led_notification_task,
                                  &led_notify,
                                  portMAX_DELAY) == pdPASS)
                {
                    if (led_notify == ControllerNotification::NOTIFY_DATA)
                    {
                        if (xQueueReceive(queue_led_colour_data,
                                          &led_colour,
                                          portMAX_DELAY) == pdPASS)
                        {
                            led_control.setLEDColour(led_colour);
                        }
                    }
                    else
                    {
                        led_control.setLEDFunction(led_notify);
                    }
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

[[noreturn]] void voltage_monitoring_process_job(void *unused_arg)
{
    ControllerState battery_voltage_monitoring_state =
        ControllerState::STATE_IDLE;
    ControllerState motor_voltage_monitoring_state =
        ControllerState::STATE_IDLE;

    enum ControllerNotification voltage_monitoring_notify =
        ControllerNotification::NOTIFY_BOOT;

    // TODO: Lots of duplication here, can we refactor this?
    while (true)
    {
        battery_voltage_monitoring_state =
            battery_voltage_monitoring.processJob(xTaskGetTickCount());

        switch (battery_voltage_monitoring_state)
        {
            case ControllerState::STATE_IDLE:
            {
                // Voltage monitoring is idle, no action required
                break;
            }
            case ControllerState::STATE_NEW_DATA:
            {
                // Log the current voltage when new data is available
                struct VoltageMonitorData voltage_monitor_data =
                    battery_voltage_monitoring.getVoltageData();
                if (voltage_monitor_data.state ==
                    VoltageBoundsCheckState::VOLTAGE_STATE_OUTSIDE_BOUNDS)
                {
                    LOG_DATA("Battery voltage out of bounds: %.2fV",
                             voltage_monitor_data.voltage);
                    voltage_monitoring_notify =
                        ControllerNotification::NOTIFY_ERROR;
                    xQueueSendToBack(queue_buzzer_notification_task,
                                     &voltage_monitoring_notify,
                                     0);

                    if (voltage_monitor_data.voltage <
                        CX_BATTERY_VOLTAGE_THRESHOLD_LOW)
                    {
                        // Implement some power down mechanism based on
                        // battery voltage being low, call
                        // triggerPowerDownProcess() in the power_control
                        // library
                        LOG_WARN(
                            "Battery voltage too low, triggering power down "
                            "process!");
                        power_control.triggerPowerDownProcess();
                    }
                }
                else
                {
                    voltage_monitoring_notify =
                        ControllerNotification::NOTIFY_DATA;
                    enum LEDColourNames led_colour =
                        LEDColourNames::LED_COLOUR_WHITE;

                    if (Utils::isNumberWithinBounds<float>(
                            voltage_monitor_data.voltage,
                            CX_BATTERY_VOLTAGE_THRESHOLD_LOW,
                            CX_BATTERY_VOLTAGE_THRESHOLD_MID_LOW))
                    {
                        led_colour = LEDColourNames::LED_COLOUR_RED;
                    }
                    else if (Utils::isNumberWithinBounds<float>(
                                 voltage_monitor_data.voltage,
                                 CX_BATTERY_VOLTAGE_THRESHOLD_MID_LOW,
                                 CX_BATTERY_VOLTAGE_THRESHOLD_MID_HIGH))
                    {
                        led_colour = LEDColourNames::LED_COLOUR_ORANGE;
                    }
                    else if (Utils::isNumberWithinBounds<float>(
                                 voltage_monitor_data.voltage,
                                 CX_BATTERY_VOLTAGE_THRESHOLD_MID_HIGH,
                                 CX_BATTERY_VOLTAGE_THRESHOLD_HIGH))
                    {
                        led_colour = LEDColourNames::LED_COLOUR_GREEN;
                    }

                    xQueueSendToBack(queue_led_colour_data, &led_colour, 0);
                }

                xQueueSendToBack(queue_led_notification_task,
                                 &voltage_monitoring_notify,
                                 0);
                break;
            }
            case ControllerState::STATE_BUSY:
            {
                break;
            }
            default:
                break;
        }

        motor_voltage_monitoring_state =
            motor_voltage_monitoring.processJob(xTaskGetTickCount());

        switch (motor_voltage_monitoring_state)
        {
            case ControllerState::STATE_IDLE:
            {
                // Voltage monitoring is idle, no action required
                break;
            }
            case ControllerState::STATE_NEW_DATA:
            {
                // Log the current voltage when new data is available
                struct VoltageMonitorData voltage_monitor_data =
                    motor_voltage_monitoring.getVoltageData();
                if (voltage_monitor_data.state ==
                    VoltageBoundsCheckState::VOLTAGE_STATE_OUTSIDE_BOUNDS)
                {
                    LOG_DATA("Motor voltage out of bounds: %.2fV",
                             voltage_monitor_data.voltage);
                    voltage_monitoring_notify =
                        ControllerNotification::NOTIFY_ERROR;
                    xQueueSendToBack(queue_buzzer_notification_task,
                                     &voltage_monitoring_notify,
                                     0);
                }
                else
                {
                    voltage_monitoring_notify =
                        ControllerNotification::NOTIFY_DATA;
                }

                xQueueSendToBack(queue_led_notification_task,
                                 &voltage_monitoring_notify,
                                 0);
                break;
            }
            case ControllerState::STATE_BUSY:
            {
                break;
            }
            default:
                break;
        }

        // Delay to allow other tasks to execute
        vTaskDelay(voltage_monitoring_job_delay);
    }
}

[[noreturn]] void power_control_process_job(void *unused_arg)
{
    ControllerState power_control_state = ControllerState::STATE_IDLE;

    enum ControllerNotification power_control_notify =
        ControllerNotification::NOTIFY_BOOT;

    while (true)
    {
        power_control_state = power_control.processJob(xTaskGetTickCount());

        switch (power_control_state)
        {
            case ControllerState::STATE_IDLE:
            {
                break;
            }
            case ControllerState::STATE_NEW_DATA:
            {
                if (power_control.isPowerDownTriggered())
                {
                    power_control_notify =
                        ControllerNotification::NOTIFY_POWER_DOWN;
                }
                else
                {
                    power_control_notify = ControllerNotification::NOTIFY_INFO;
                }

                xQueueSendToBack(queue_led_notification_task,
                                 &power_control_notify,
                                 0);
                xQueueSendToBack(queue_buzzer_notification_task,
                                 &power_control_notify,
                                 0);

                break;
            }
            case ControllerState::STATE_BUSY:
            {
                break;
            }
            default:
                break;
        }

        // Delay to allow other tasks to execute
        vTaskDelay(power_control_job_delay_ms);
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

    LOG_INFO("Setting up peripherals...");
    setup();
    LOG_INFO("Setting up peripherals... OK!");

    // Create a FreeRTOS timer with a period slightly shorter than the watchdog
    // timeout
    TimerHandle_t timer_watchdog_service =
        xTimerCreate("WatchdogTimer",           // Timer name
                     watchdog_job_delay_ms,     // Timer period in ticks
                     pdTRUE,                    // Auto-reload timer
                     (void *)0,                 // Timer ID (not used here)
                     watchdog_timer_callback);  // Callback function

    if (timer_watchdog_service == NULL)
    {
        LOG_ERROR("Failed to create FreeRTOS timer!");
    }
    else
    {
        if (xTimerStart(timer_watchdog_service, 0) != pdPASS)
        {
            LOG_ERROR("Failed to start FreeRTOS timer!");
        }
        else
        {
            LOG_INFO("FreeRTOS timer started successfully!");
        }
    }

    // FROM 1.0.1 Store handles referencing the tasks; get return values
    // NOTE Arg 3 is the stack depth -- in words, not bytes
    BaseType_t led_status = xTaskCreate(led_process_job,
                                        "GPIO_LED_TASK",
                                        256,
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

    BaseType_t voltage_monitoring_status =
        xTaskCreate(voltage_monitoring_process_job,
                    "VOLTAGE_MONITORING_JOB_TASK",
                    256,
                    NULL,
                    job_priority_voltage_monitoring,
                    &voltage_monitoring_task_handle);

    BaseType_t power_control_status = xTaskCreate(power_control_process_job,
                                                  "POWER_CONTROL_JOB_TASK",
                                                  256,
                                                  NULL,
                                                  job_priority_power_control,
                                                  &power_control_task_handle);

    // Set up the event queue
    queue_motor_control_data = xQueueCreate(2, sizeof(struct MotorControlData));
    queue_led_notification_task =
        xQueueCreate(1, sizeof(enum ControllerNotification));
    queue_buzzer_notification_task =
        xQueueCreate(1, sizeof(enum ControllerNotification));
    queue_led_colour_data = xQueueCreate(1, sizeof(LEDColourNames));
    queue_power_control_data =
        xQueueCreate(1, sizeof(enum ControllerNotification));

    // Start the FreeRTOS scheduler if all tasks are created successfully
    if (led_status == pdPASS && tmc_status == pdPASS &&
        joystick_status == pdPASS && buzzer_status == pdPASS &&
        voltage_monitoring_status == pdPASS &&
        power_control_status == pdPASS)  // Add this condition
    {
        vTaskStartScheduler();
    }

    // We should never get here, but just in case...
    while (true)
    {
        // NOP
    }
}

// Callback function for the repeating timer
void watchdog_timer_callback(__unused TimerHandle_t xTimer)
{
    watchdog_update();  // Feed the watchdog timer
}