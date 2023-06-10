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

static volatile bool is_callback_complete = false;

static bool driver_can_be_enabled = false;

void callback(TMC2300TypeDef *tmc2300, ConfigState cfg_state)
{
    UNUSED(tmc2300);

    if (cfg_state == CONFIG_RESET)
    {
        driver_can_be_enabled = false;
        // Configuration reset completed
        // Change hardware preset registers here

        // m_ihold_irun.ihold = 1;
        // m_ihold_irun.irun = 4;
        // m_ihold_irun.iholddelay = 2;

        // // Lower the default run and standstill currents
        // tmc2300_writeInt(tmc2300, m_ihold_irun.address, m_ihold_irun.sr);

        // TODO: Update register values also to reflect state
        // Lower the default run and standstill currents
        tmc2300_writeInt(tmc2300, TMC2300_IHOLD_IRUN, 0x00010402);
    }
    else
    {
        printf("TMC is ready for action!\n");
        // Configuration restore complete
        is_callback_complete = true;
        // The driver may only be enabled once the configuration is done
        driver_can_be_enabled = true;
    }
}

TMCControl::TMCControl()
{
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

        // Generic I/O setup
        gpio_init(TMC_PIN_ENABLE);
        gpio_set_dir(TMC_PIN_ENABLE, GPIO_OUT);

        gpio_init(TMC_PIN_N_STANDBY);
        gpio_set_dir(TMC_PIN_N_STANDBY, GPIO_OUT);

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
    m_gconf.extcap = true;       // external capacitor available
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
        printf("Reset = %d\n", m_gstat.reset);
        printf("Error, shut down = %d\n", m_gstat.drv_err);
        printf("Low supply voltage = %d\n", m_gstat.u3v5);
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
    m_ihold_irun.ihold = 0;       // Standstill current
    m_ihold_irun.irun = 10;       // Motor run current
    m_ihold_irun.iholddelay = 2;  // Number of clock cycles for motor power down
                                  // after standstill detected
    tmc2300_writeInt(&tmc2300, m_ihold_irun.address, m_ihold_irun.sr);

    /* Register: VACTUAL
     * What: Motor velocity value in +-(2^23)-1 [μsteps / t]
     * Use: Set to 0 for STEP/DIR operation, otherwise provide value for UART
     * control to set motor velocity.
     */
    m_vactual.sr = 10000U;
    tmc2300_writeInt(&tmc2300, m_vactual.address, m_vactual.sr);

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
    m_pwmconf.pwm_autograd = false;  // PWM automatic gradient adaptation
    m_pwmconf.freewheel =
        1;                  // Standstill option when motor current setting is 0
    m_pwmconf.pwm_reg = 4;  // Regulation loop gradient
    m_pwmconf.pwm_lim =
        12;  // PWM automatic scale amplitude limit when switching on
    tmc2300_writeInt(&tmc2300, m_pwmconf.address, m_pwmconf.sr);
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

void TMCControl::updateMovementDynamics(int32_t velocity_delta,
                                        int8_t direction)
{
    int32_t _velocity = m_vactual.sr + velocity_delta;

    if (_velocity < 0 && direction == -1)
    {
        // Do nothing
        ;
    }
    if (_velocity > 0 && direction == 1)
    {
        // Do nothing
        ;
    }
    if (_velocity < 0 && direction == 1)
    {
        // Invert direction
        _velocity = -_velocity;
    }
    if (_velocity > 0 && direction == -1)
    {
        // Invert direction
        _velocity = -_velocity;
    }
    if (direction == 0)
    {
        // Stop motor
        _velocity = 0;
    }

    move(_velocity);
}

void TMCControl::stop()
{
    move(0);
    enableDriver(false);
}

void TMCControl::move(int32_t velocity)
{
    if (m_vactual.sr != velocity)
    {
        if (abs(velocity) > VELOCITY_MAX_STEPS_PER_SECOND)
        {
            Utils::log_warn("Max motor velocity reached!");

            velocity = VELOCITY_MAX_STEPS_PER_SECOND;
        }
        printf("New velocity: %d\n", velocity);
        m_vactual.sr = velocity;
        enableDriver(velocity == 0 ? false : true);
        tmc2300_writeInt(&tmc2300, m_vactual.address, m_vactual.sr);
    }
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

void TMCControl::enableUartPins(bool enablePins)
{
    if (enablePins && !m_uart_pins_enabled)
    {
        Utils::log_info("UART pins enable");
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
    else if (!enablePins)
    {
        Utils::log_info("UART pins disable");
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
    sleep_ms(10);
}

struct TMCData TMCControl::getTMCData()
{
    m_tmc.control_state = ControllerState::STATE_READY;
    return m_tmc;
}

enum ControllerState TMCControl::processJob(uint32_t tick_count)
{
    tmc2300_periodicJob(&tmc2300, tick_count);

    if (is_callback_complete &&
        m_tmc.control_state == ControllerState::STATE_BUSY)
    {
        // TODO: Unsure if the callback is triggered for every TMC register
        // write... If so, this logic needs re-writing
        m_tmc.control_state = ControllerState::STATE_READY;
    }

    // if(DIAG_PIN changes state)
    // {
    //     m_tmc.control_state = ControllerState::STATE_NEW_DATA;
    // }

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
    gpio_put(TMC_PIN_N_STANDBY, enable_standby ? 0 : 1);

    // Update the APIs internal standby state
    tmc2300_setStandby(&tmc2300, enable_standby ? 1 : 0);
}

void TMCControl::enableDriver(bool enable_driver)
{
    bool _enable_driver = (bool)(driver_can_be_enabled && enable_driver);

    printf("enable driver: %d\n", _enable_driver);
    gpio_put(TMC_PIN_ENABLE, _enable_driver ? 1 : 0);

    // TODO: Check if necessary
    sleep_ms(10);
}
