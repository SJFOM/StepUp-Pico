/**
 * @file tmc_control.cpp
 * @author SJFOM (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-02-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../include/tmc_control.hpp"

static TMC2300TypeDef tmc2300;
static ConfigurationTypeDef tmc2300_config;

// Static non-class-member callback variables
static volatile bool s_is_callback_complete = false;
static volatile bool s_diag_event = false;

// Static non-class-member variables
static bool driver_can_be_enabled = false;

static void tmc_diag_callback();

void callback(TMC2300TypeDef *tmc2300, ConfigState cfg_state)
{
    UNUSED(tmc2300);

    if (cfg_state == CONFIG_RESET)
    {
        driver_can_be_enabled = false;
        // Configuration reset completed
        // Change hardware preset registers here

        // TODO: Update register values also to reflect state
        // m_ihold_irun.ihold = 1;
        // m_ihold_irun.irun = 4;
        // m_ihold_irun.iholddelay = 2;
        // Lower the default run and standstill currents
        tmc2300_writeInt(tmc2300, TMC2300_IHOLD_IRUN, 0x00010402);
    }
    else
    {
        Utils::log_info("TMC is ready for use");
        // Configuration restore complete
        s_is_callback_complete = true;
        // The driver may only be enabled once the configuration is done
        driver_can_be_enabled = true;
    }
}

TMCControl::TMCControl()
{
    m_motor_move_state = MotorMoveState::MOTOR_IDLE;
    m_init_success = false;
    m_uart_pins_enabled = false;
    driver_can_be_enabled = false;
    m_tmc.control_state = ControllerState::STATE_IDLE;
}
TMCControl::~TMCControl()
{
    deinit();
}

bool TMCControl::init()
{
    // Avoid multiple calls to this method performing the same configuration
    // steps
    if (false == m_init_success)
    {
        // Initially set as true
        bool _init_routine_success = true;

        gpio_init(TMC_ENABLE_PIN);
        gpio_set_dir(TMC_ENABLE_PIN, GPIO_OUT);

        gpio_init(TMC_N_STANDBY_PIN);
        gpio_set_dir(TMC_N_STANDBY_PIN, GPIO_OUT);

        // Configure DIAG pin as interrupt
        gpio_init(TMC_DIAG_PIN);
        gpio_set_input_enabled(TMC_DIAG_PIN, true);
        gpio_pull_up(TMC_DIAG_PIN);

        // Initialize CRC calculation for TMC2300 UART datagrams
        if (!(tmc_fillCRC8Table(0x07, true, 0) == 1))
        {
            Utils::log_info("Error: Fill CRC");
            _init_routine_success &= false;
        }

        // Configure TMC2300 IC and default register states
        tmc2300_init(&tmc2300,
                     TMC_UART_CHANNEL,
                     &tmc2300_config,
                     tmc2300_defaultRegisterResetState);

        tmc2300_setSlaveAddress(&tmc2300, TMC_UART_SLAVE_ADDRESS);

        tmc2300_setCallback(&tmc2300, callback);

        // Set up our UART with the required speed.
        enableUartPins(true);
        uint _baud = uart_init(TMC_UART_ID, TMC_BAUD_RATE);
        if (!(_baud >= TMC_BAUD_RATE))
        {
            Utils::log_warn("UART init");
            Utils::log_warn((string) "Actual BAUD: " + std::to_string(_baud));
            _init_routine_success &= false;
        }

        setStandby(false);
        enableDriver(false);

        m_tmc.control_state = ControllerState::STATE_BUSY;

        // Complete checks and store init routine success value
        m_init_success = _init_routine_success;
    }

    return m_init_success;
}

void TMCControl::deinit()
{
    // Disable DIAG pin interrupt function
    enableTMCDiagInterrupt(false);
    // Reset the tmc2300 to its default values and state
    tmc2300_reset(&tmc2300);

    // De-initialise the uart peripheral
    uart_deinit(TMC_UART_ID);

    m_tmc.control_state = ControllerState::STATE_IDLE;
    m_init_success = false;
}

void TMCControl::defaultConfiguration()
{
    // NOTE: Where possible, default Datasheet values have been applied to the
    // registers below.

    /* Register: GCONF
     * What: Global configuration flags
     * User: Set these to determine high-level system function
     */
    m_gconf.sr = tmc2300_readInt(&tmc2300, m_gconf.address);
    m_gconf.extcap =
        true;  // external capacitor available, avoid switching delays
    m_gconf.shaft = false;       // don't invert motor direction
    m_gconf.diag_index = false;  // DIAG output normal
    m_gconf.diag_step = false;   // DIAG output normal
    m_gconf.multistep_filt =
        true;                   // pulse generator optimized for higher speeds
    m_gconf.test_mode = false;  // Normal operation
    tmc2300_writeInt(&tmc2300, m_gconf.address, m_gconf.sr);

    /* Register: GSTAT
     * What: Error flags reflection of IC's operating status (0 = OK, 1 = ERROR)
     * Use: Read values to understand state, write to clear error flags
     */
    m_gstat.reset = false;    // Indicates the IC has been reset
    m_gstat.drv_err = false;  // Indicates the driver has been shut down due to
                              // error, read DRV_STATUS for reason
    m_gstat.u3v5 = false;     // Supply voltage sinks below 3.5V
    m_gstat.sr = tmc2300_readInt(&tmc2300, m_gstat.address);
    if (m_gstat.sr)
    {
        Utils::log_info("TMC2300 GSTAT register diagnostics:");
        Utils::log_info((string) "\t- Reset?: " +
                        std::to_string(m_gstat.reset));
        Utils::log_info((string) "\t- Driver shutdown due to error?: " +
                        std::to_string(m_gstat.drv_err));
        Utils::log_info((string) "\t- Low supply voltage?: " +
                        std::to_string(m_gstat.u3v5));
    }

    /* Register: IOIN_t
     * What: Read state of available TMC2300 input pins
     * Use: Read values to understand operating state of motor movements
     */
    m_ioin.en = 0;         // EN pin (0=disable, 1=enable)
    m_ioin.nstdby = 0;     // NSTBY pin (0=standby, 1=enable)
    m_ioin.ad0 = 0;        // AD0 pin (UART address LSB)
    m_ioin.ad1 = 0;        // AD1 pin (UART address MSB)
    m_ioin.diag = 0;       // Diag pin
    m_ioin.stepper = 0;    // STEPPER pin (1: UART interface ON)
    m_ioin.pdn_uart = 0;   // PDN_UART pin
    m_ioin.mode = 0;       // MODE pin (0: UART controller operation)
    m_ioin.step = 0;       // STEP pin
    m_ioin.dir = 0;        // DIR pin
    m_ioin.comp_a1a2 = 0;  // COMP_A1A2 pin: 1: during LS passive braking: A1
                           // voltage > A2 voltage
    m_ioin.comp_b1b2 = 0;  // COMP_B1B2 pinL 1: during LS passive braking: B1
                           // voltage > B2 voltage
    m_ioin.sr = tmc2300_readInt(&tmc2300, m_ioin.address);

    /* Register: IHOLD_IRUN
     * What: Configure run and hold currents
     * Use: Control motor power consumption and ability to drive loads
     */
    m_ihold_irun.sr = tmc2300_readInt(&tmc2300, m_ihold_irun.address);
    m_ihold_irun.ihold = DEFAULT_IHOLD_VALUE;  // Standstill current
    m_ihold_irun.irun = DEFAULT_IRUN_VALUE;    // Motor run current
    m_ihold_irun.iholddelay = 2;  // Number of clock cycles for motor power down
                                  // after standstill detected
    tmc2300_writeInt(&tmc2300, m_ihold_irun.address, m_ihold_irun.sr);

    /* Register: VACTUAL
     * What: Motor velocity value in +-(2^23)-1 [μsteps / t]
     * Use: Set to 0 for STEP/DIR operation, otherwise provide value for UART
     * control to set motor velocity.
     */
    m_vactual.sr = VELOCITY_STARTING_STEPS_PER_SECOND;
    tmc2300_writeInt(&tmc2300, m_vactual.address, m_vactual.sr);

    /* Register: SGTHRS
     * What: Detection threshold for stall - when SG_VAL falls below 2x this.
     * number
     * Use: Set between 0..255, higher value = higher sensitivity to
     * stall.
     */
    m_sgthrs.sr = DEFAULT_SGTHRS_VALUE;
    tmc2300_writeInt(&tmc2300, m_sgthrs.address, m_sgthrs.sr);

    /* Register: TCOOLTHRS
     * What: Lower threshold velocity for switching on CoolStep and Stallgaurd.
     * Use: Velocity in steps/second at which to switch on this feature
     */
    // TODO: Update with sane velocity at which to switch mode.
    m_tcoolthrs.sr = 10000U;
    tmc2300_writeInt(&tmc2300, m_tcoolthrs.address, m_tcoolthrs.sr);

    /* Register: COOLCONF
     * What: Configure how CoolStep is employed wrt to Stallgaurd values
     * Use: See datasheet, page 26
     */
    // TODO: Update with stallgaurd value thresholds once known
    m_coolconf.sr = 0;
    m_coolconf.seup = 0;  // Step width: 1
    m_coolconf.sedn = 0;  // SG measurements per decrement: 32
    m_coolconf.semin = 1;
    m_coolconf.semax = 15;
    m_coolconf.seimin = 0;
    if (m_ihold_irun.irun >= 20U)
    {
        m_coolconf.seimin = 1;
    }
    tmc2300_writeInt(&tmc2300, m_coolconf.address, m_coolconf.sr);

    /* Register: CHOPCONF
     * What: Generic chopper algorithm configuration
     * Use: Configuring output stage power management and step resolution
     * parameters
     */
    m_chopconf.sr = tmc2300_readInt(&tmc2300, m_chopconf.address);
    m_chopconf.enabledrv = true;  // Driver enable (0=disable, 1=enable)
    m_chopconf.tbl = 2;           // Comparator blank time in clock-counts
    m_chopconf.mres = 3;          // Microstep setting (0=256 μsteps)
    m_chopconf.intpol = true;     // Interpolation to 256 μsteps
    m_chopconf.dedge = false;     // Enable double edge step pulses
    m_chopconf.diss2g = false;  // Short to GND protection (0=enable, 1=disable)
    m_chopconf.diss2vs =
        false;  // Low side short protection (0=enable, 1=disable)
    tmc2300_writeInt(&tmc2300, m_chopconf.address, m_chopconf.sr);

    /* Register: DRV_STATUS
     * What: Driver status flags and current level read back
     * Use: Read electrical and temperature diagnostics of TMC2300 IC
     */
    m_drv_status.sr = tmc2300_readInt(&tmc2300, m_drv_status.address);

    /* Register: DRV_STATUS
     * What: Driver status flags and current level read back
     * Use: Read electrical and temperature diagnostics of TMC2300 IC
     */
    m_pwmconf.sr = tmc2300_readInt(&tmc2300, m_pwmconf.address);
    m_pwmconf.pwm_ofs = 36;   // User defined PWM amplitude offset
    m_pwmconf.pwm_grad = 16;  // Velocity dependent gradient for PWM amplitude
    m_pwmconf.pwm_freq = 0;   // PWM frequency selection
    m_pwmconf.pwm_autoscale = true;  // PWM automatic amplitude scaling
    m_pwmconf.pwm_autograd = true;   // PWM automatic gradient adaptation
    m_pwmconf.freewheel =
        1;                  // Standstill option when motor current setting is 0
    m_pwmconf.pwm_reg = 4;  // Regulation loop gradient
    m_pwmconf.pwm_lim =
        12;  // PWM automatic scale amplitude limit when switching on
    tmc2300_writeInt(&tmc2300, m_pwmconf.address, m_pwmconf.sr);

    /* Register: PWM_SCALE
     * What: Results of StealthChop amplitude regulator
     * Use: These values can be used to monitor automatic PWM amplitude scaling
     */
    m_pwm_scale.sr = tmc2300_readInt(&tmc2300, m_pwm_scale.address);

    // Enable the TMC interrupt and associated callback function
    enableTMCDiagInterrupt(true);
    gpio_add_raw_irq_handler(TMC_DIAG_PIN, &tmc_diag_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void TMCControl::setCurrent(uint8_t i_run, uint8_t i_hold)
{
    if (i_run > 31U)
    {
        printf("i_run value exceeded - limiting to 31\n");
        i_run = 31U;
    }
    if (i_hold > 31U)
    {
        printf("i_hold value exceeded - limiting to 31\n");
        i_hold = 31U;
    }

    m_ihold_irun.irun = i_run;
    m_ihold_irun.ihold = i_hold;
    tmc2300_writeInt(&tmc2300, m_ihold_irun.address, m_ihold_irun.sr);
}

void TMCControl::updateCurrent(uint8_t i_run_delta)
{
    uint8_t _i_run = m_ihold_irun.irun + i_run_delta;

    setCurrent(_i_run, m_ihold_irun.ihold);
}

// Steps
/* 1 - Record current speed
 * 2 - Some way of knowing that we are idle -> moving -> need to accelerate
 *   - Store speed step increment value (1000U)
 *   - Store state of motor: idle, moving etc..
 * 3 - If changing velocity due to user input (i.e. velocity_delta != 0) then we
 * need to leave accel-decel mode
 *
 * Thought... Do we need to have separate modes? Why don't we have
 * updateMovementDynamics change the target_velocity and make it the job of
 * processJob to accelerate towards the velocity? Hmmm.....
 */

void TMCControl::updateMovementDynamics(int32_t velocity_delta,
                                        int8_t direction)
{
    // TODO: This method is quite specific to updating based on a delta -
    // suggest renaming or splitting into two separate methods
    if (direction == 0)
    {
        enableDriver(false);
    }
    else
    {
        int32_t ramp_velocity = abs(m_vactual.sr);
        ramp_velocity *= direction;

        // We do not wish to allow a velocity update to make the motor stop, the
        // direction flag should control this. Instead, we want to enforce a
        // call to resetMotorDynamics or move(0) to have this effect.
        if ((ramp_velocity + velocity_delta) != 0)
        {
            ramp_velocity += velocity_delta;
        }

        m_target_velocity = ramp_velocity;

        enableDriver(true);
    }
}

void TMCControl::resetMovementDynamics()
{
    move(VELOCITY_STARTING_STEPS_PER_SECOND);
    enableDriver(false);
    setCurrent(DEFAULT_IRUN_VALUE, DEFAULT_IHOLD_VALUE);
}

void TMCControl::move(int32_t velocity)
{
    // Utils::log_debug((string) "velocity: " + std::to_string(velocity));
    if (abs(velocity) > VELOCITY_MAX_STEPS_PER_SECOND)
    {
        Utils::log_warn("Max motor velocity reached!");

        velocity = VELOCITY_MAX_STEPS_PER_SECOND;
    }
    // printf("New velocity: %d\n", velocity);
    m_vactual.sr = velocity;
    enableDriver(velocity == 0 ? false : true);
    tmc2300_writeInt(&tmc2300, m_vactual.address, m_vactual.sr);
}

uint8_t TMCControl::getChipID()
{
    // Check we have established a connection with the TMC2300 by reading its
    // serial number
    int32_t tmc_version = tmc2300_readInt(&tmc2300, TMC2300_IOIN);
    tmc_version =
        ((tmc_version & TMC2300_VERSION_MASK) >> TMC2300_VERSION_SHIFT);
    return tmc_version;
}

void TMCControl::enableUartPins(bool enable_pins)
{
    if (enable_pins && !m_uart_pins_enabled)
    {
        Utils::log_info("TMC - UART pins enable");
        // Set the TX and RX pins by using the function select on the GPIO
        // Set datasheet for more information on function select
        gpio_init(TMC_UART_TX_PIN);
        gpio_set_dir(TMC_UART_TX_PIN, GPIO_OUT);
        gpio_init(TMC_UART_RX_PIN);
        gpio_set_dir(TMC_UART_RX_PIN, GPIO_IN);
        gpio_set_function(TMC_UART_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(TMC_UART_RX_PIN, GPIO_FUNC_UART);

        m_uart_pins_enabled = true;
    }
    else if (!enable_pins)
    {
        Utils::log_info("TMC - UART pins disable");
        // Set the UART pins as standard IO's
        gpio_set_function(TMC_UART_TX_PIN, GPIO_FUNC_SIO);
        gpio_set_function(TMC_UART_RX_PIN, GPIO_FUNC_SIO);

        // Disable any pull-ups/downs on the IO pins
        gpio_disable_pulls(TMC_UART_TX_PIN);
        gpio_disable_pulls(TMC_UART_RX_PIN);

        // Set the UART pins as inputs
        gpio_set_input_enabled(TMC_UART_TX_PIN, true);
        gpio_set_input_enabled(TMC_UART_RX_PIN, true);

        m_uart_pins_enabled = false;
    }

    // TODO: Figure out if actually needed...
    sleep_ms(5);
}

TMCDiagnostics TMCControl::readTMCDiagnostics()
{
    // Start with a blank TMCDiagnostics with all flags set to false
    TMCDiagnostics tmc_diag;
    m_drv_status.sr = tmc2300_readInt(&tmc2300, m_drv_status.address);

    m_pwm_scale.sr = tmc2300_readInt(&tmc2300, m_pwm_scale.address);

    // Utils::log_debug((string) "PWM_SCALE_SUM: " +
    //                  std::to_string(m_pwm_scale.pwm_scale_sum));

    if (m_drv_status.otpw || m_drv_status.ot || m_drv_status.t120 ||
        m_drv_status.t150)
    {
        tmc_diag.overheating = true;
    }

    // The TMC2300 datasheet mentions the motor should be moving "slowly" for
    // this open-circuit detection to be valid - see description of the ola &
    // olb flags in the DRV_STATUS register map
    // NOTE: If we have a known PWM_SCALE_SUM value at "low speed" for the motor
    // and we suddenly detect a wildly different value here then we might also
    // consider relying on using that as additional information for open-load
    // detection.
    if ((m_drv_status.ola || m_drv_status.olb) &&
        (abs(m_vactual.sr) <= VELOCITY_SLOW_SPEED_STEPS_PER_SECOND))
    {
        tmc_diag.open_circuit = true;
    }

    if (m_drv_status.s2ga || m_drv_status.s2gb || m_drv_status.s2vsa ||
        m_drv_status.s2vsb)
    {
        tmc_diag.short_circuit = true;
    }

    // FIXME: Seems to trigger whenever a button press occurs, should filter
    // this combination of events at the main.cpp level.
    if (m_drv_status.stst)
    {
        tmc_diag.stall_detected = true;
    }

    if (m_drv_status.sr & m_drv_status.error_bit_mask)
    {
        tmc_diag.normal_operation = false;
    }

    return tmc_diag;
}

struct TMCData TMCControl::getTMCData()
{
    // TODO: Should we be re-setting the state to READY here or should that be
    // handled by the controlling code in main.cpp?
    m_tmc.control_state = ControllerState::STATE_READY;
    m_tmc.diag = readTMCDiagnostics();
    return m_tmc;
}

static uint16_t sg_val_total;

enum ControllerState TMCControl::processJob(uint32_t tick_count)
{
    static unsigned int process_count = 0;
    tmc2300_periodicJob(&tmc2300, tick_count);

    if (s_is_callback_complete &&
        m_tmc.control_state == ControllerState::STATE_BUSY)
    {
        // TODO: Unsure if the callback is triggered for every TMC register
        // write... If so, this logic needs re-writing
        m_tmc.control_state = ControllerState::STATE_READY;
    }

    sg_val_total += tmc2300_readInt(&tmc2300, TMC2300_SG_VALUE);

    // Stall detection, over temperature & short-circuit detection are all
    // mapped to the DIAG pin. However, open-circuit flags must be polled and
    // are not mapped to the DIAG pin flag.
    if (s_diag_event || (process_count++ > 10))
    {
        s_diag_event = false;
        m_tmc.control_state = ControllerState::STATE_NEW_DATA;

        // TODO: Remove, just for diagnostics
        uint32_t sg_value = tmc2300_readInt(&tmc2300, TMC2300_SG_VALUE);
        IHOLD_IRUN_t irun_ihold;
        irun_ihold.sr = tmc2300_readInt(&tmc2300, TMC2300_IHOLD_IRUN);
        uint8_t motor_effort_percent = ((100 * (510 - sg_value)) / 510);
        m_ioin.sr = tmc2300_readInt(&tmc2300, m_ioin.address);
        m_drv_status.sr = tmc2300_readInt(&tmc2300, m_drv_status.address);
        m_ihold_irun.sr = tmc2300_readInt(&tmc2300, TMC2300_IHOLD_IRUN);
        uint8_t stall = 0;
        uint8_t diag = 0;
        if (m_ioin.diag)
        {
            diag = 1;
            if (m_drv_status.stst)
            {
                stall = 1;
            }
        }
        // printf("SG: %d\n", sg_value);
        // if (sg_value < 30U)
        // {
        // printf("High motor load: %d - %d %%\n", sg_value,
        // motor_effort_percent);
        // }
        printf(">sg_live: %d\n", sg_value);
        printf(">sg_avg: %d\n", sg_val_total / process_count);
        printf(">vel:%d\n", m_vactual.sr);
        printf(">diag: %d\n", diag);
        printf(">diag_pin: %d\n", gpio_get(TMC_DIAG_PIN));
        printf(">stall: %d\n", stall);
        printf(">thresh: %d\n", m_sgthrs.sr);
        printf(">drv_status: %d\n",
               m_drv_status.sr & m_drv_status.error_bit_mask);
        printf(">irun: %d\n", m_ihold_irun.irun);
        sg_val_total = 0;
        process_count = 0;
    }

    // Ramp profile using internal TMC step generator
    switch (m_motor_move_state)
    {
        static int32_t ramp_velocity = 0;
        case (MOTOR_IDLE):
            break;
        case (MOTOR_IDLE_TO_MOVING):
        {
            m_vactual.sr = VELOCITY_MAX_STEPS_PER_SECOND;
            m_motor_move_state = MotorMoveState::MOTOR_MOVING;
        }
        case (MOTOR_MOVING):
        {
            if (m_vactual.sr != m_target_velocity)
            {
                if (abs(m_vactual.sr) > abs(m_target_velocity))
                {
                    // If we are moving faster than the target velocity then we
                    // can safely jump to using the target velocity. Use of the
                    // ramp profile makes sense when we are trying to increase
                    // our velocity to a VMAX vs the other way around.
                    ramp_velocity = m_target_velocity;
                }
                else
                {
                    // If we are not yet at our target velocity, increment our
                    // ramp velocity by a set increment value in the direction
                    // of motor rotation.
                    if (m_target_velocity < 0)
                    {
                        ramp_velocity -=
                            VELOCITY_RAMP_INCREMENT_STEPS_PER_SECOND;
                    }
                    else
                    {
                        ramp_velocity +=
                            VELOCITY_RAMP_INCREMENT_STEPS_PER_SECOND;
                    }
                }
                // printf("%d -> %d -> %d\n",
                //        m_vactual.sr,
                //        ramp_velocity,
                //        m_target_velocity);
                move(ramp_velocity);
            }
            break;
        }
        case (MOTOR_MOVING_TO_IDLE):
        {
            // printf("Moving -> Idle\n");
            m_motor_move_state = MotorMoveState::MOTOR_IDLE;
            break;
        }
        default:
            break;
    };

    return m_tmc.control_state;
}

extern "C" void tmc2300_readWriteArray(uint8_t channel,
                                       uint8_t *data,
                                       size_t writeLength,
                                       size_t readLength)
{
    // This is needed to wait for the TMC2300 to be ready to receive and/or
    // reply with data, otherwise the Pico's UART peripheral has a hard time
    // finding the data...
    // FIXME: This can eventually become interrupt driven
    sleep_ms(1);

    // Write data buffer
    uart_write_blocking(TMC_UART_ID, data, writeLength);
    // Read out echo'd data to a nullptr (don't care)
    // UART is using a fifo so need to clear it for every read event
    uart_read_blocking(TMC_UART_ID, nullptr, writeLength);

    // If no reply data is expected abort here
    if (readLength == 0) return;

    // Read the reply data
    uart_read_blocking(TMC_UART_ID, data, readLength);
}

extern "C" uint8_t tmc2300_CRC8(uint8_t *data, size_t length)
{
    // tmc_CRC8 is a generic library method so will need to be mapped
    // to the tmc2300_CRC8 extern method.
    return tmc_CRC8(data, length, 0);
}

void TMCControl::setStandby(bool enable_standby)
{
    // En/disable the UART pins depending on standby state
    enableUartPins(!enable_standby);

    if (enable_standby)
    {
        // Just entered standby -> disable the driver
        enableDriver(false);
    }

    // Set the Standby pin state - after enable so we retain control over driver
    gpio_put(TMC_N_STANDBY_PIN, enable_standby ? 0 : 1);

    // Update the APIs internal standby state
    tmc2300_setStandby(&tmc2300, enable_standby ? 1 : 0);
}

bool TMCControl::isDriverEnabled()
{
    return gpio_get(TMC_ENABLE_PIN);
}

void TMCControl::enableTMCDiagInterrupt(bool enable_interrupt)
{
    // Set up and enable the TMC DIAG pin interrupt when we have finished
    // initialising the TMC
    gpio_set_irq_enabled(TMC_DIAG_PIN,
                         GPIO_IRQ_EDGE_RISE,
                         enable_interrupt);  // monitor pin 1 connected to pin 0
}

void TMCControl::enableDriver(bool enable_driver)
{
    bool _enable_driver = (bool)(driver_can_be_enabled && enable_driver);

    if (isDriverEnabled() && !_enable_driver)
    {
        m_motor_move_state = MotorMoveState::MOTOR_MOVING_TO_IDLE;
    }
    else if (!isDriverEnabled() && _enable_driver)
    {
        m_motor_move_state = MotorMoveState::MOTOR_IDLE_TO_MOVING;
    }

    // Utils::log_debug((string) "Driver enabled: " +
    //  std::to_string(_enable_driver));
    gpio_put(TMC_ENABLE_PIN, _enable_driver ? 1 : 0);
}

/******************************/
/* Interrupt routines - START */
/******************************/

void tmc_diag_callback()
{
    if (gpio_get_irq_event_mask(TMC_DIAG_PIN) & GPIO_IRQ_EDGE_RISE)
    {
        gpio_acknowledge_irq(TMC_DIAG_PIN, GPIO_IRQ_EDGE_RISE);
        // TODO: Check if this needs de-bouncing
        s_diag_event = true;
    }
}

/****************************/
/* Interrupt routines - END */
/****************************/