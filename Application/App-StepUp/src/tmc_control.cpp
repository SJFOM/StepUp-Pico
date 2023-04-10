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
        gpio_init(PIN_TMC_ENABLE);
        gpio_set_dir(PIN_TMC_ENABLE, GPIO_OUT);

        gpio_init(PIN_TMC_N_STANDBY);
        gpio_set_dir(PIN_TMC_N_STANDBY, GPIO_OUT);

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
        uint _baud = uart_init(UART_ID, BAUD_RATE);
        if (!(_baud >= BAUD_RATE))
        {
            Utils::log_warn("UART init");
            Utils::log_warn((string) "Actual BAUD: " + std::to_string(_baud));
            _init_routine_success &= false;
        }

        setStandby(false);
        enableDriver(false);
        unsigned callback_safe_count = 0;
        while (!is_callback_complete)
        {
            if (callback_safe_count++ > 100)
            {
                // Typical count is <10, we should have high conviction here
                // that something is wrong.
                m_init_success = false;
                break;
            }
            // TODO: Potentially caught in this loop forever
            tmc2300_periodicJob(&tmc2300, 0);
            sleep_ms(10);
        }
        is_callback_complete = false;

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
    uart_deinit(UART_ID);

    m_init_success = false;
}

void TMCControl::defaultConfiguration()
{
    // NOTE: Where possible, default Datasheet values have been applied to the registers below.

    /* Register: GCONF 
    * What: Global configuration flags
    * User: Set these to determine high-level system function
    */
    m_gconf.sr = tmc2300_readInt(&tmc2300, m_gconf.address);
    m_gconf.extcap = 1; // external capacitor available
    m_gconf.shaft = 0; // don't invert motor direction
    m_gconf.diag_index = 0; // DIAG output normal
    m_gconf.diag_step = 0;  // DIAG output normal
    m_gconf.multistep_filt = 1; // pulse generator optimized for higher speeds
    m_gconf.test_mode = 0; // Normal operation
    tmc2300_writeInt(&tmc2300, m_gconf.address, m_gconf.sr);

    /* Register: GSTAT 
    * What: Error flags reflection of IC's operating status (0 = OK, 1 = ERROR)
    * Use: Read values to understand state, write to clear error flags
    */
    m_gstat.reset = 0; // Indicates the IC has been reset
    m_gstat.drv_err = 0; // Indicates the driver has been shut down due to error, read DRV_STATUS for reason
    m_gstat.u3v5 = 0; // Supply voltage sinks below 3.5V
    m_gstat.sr = tmc2300_readInt(&tmc2300, m_gstat.address);

    /* Register: IOIN_t
    * What: Read state of available TMC2300 input pins
    * Use: Read values to understand operating state of motor movements
    */
    m_ioin.en = 0; // EN pin (0=disable, 1=enable)
    m_ioin.nstdby = 0; // NSTBY pin (0=standby, 1=enable)
    m_ioin.ad0 = 0; // AD0 pin (UART address LSB)
    m_ioin.ad1 = 0; // AD1 pin (UART address MSB)
    m_ioin.diag = 0; // Diag pin
    m_ioin.stepper = 0; // STEPPER pin (1: UART interface ON)
    m_ioin.pdn_uart = 0; // PDN_UART pin
    m_ioin.mode = 0; // MODE pin (0: UART controller operation)
    m_ioin.step = 0; // STEP pin
    m_ioin.dir = 0; // DIR pin 
    m_ioin.comp_a1a2 = 0; // COMP_A1A2 pin: 1: during LS passive braking: A1 voltage > A2 voltage
    m_ioin.comp_b1b2 = 0; // COMP_B1B2 pinL 1: during LS passive braking: B1 voltage > B2 voltage
    m_ioin.sr = tmc2300_readInt(&tmc2300, m_ioin.address);

    /* Register: IHOLD_IRUN
    * What: Configure run and hold currents
    * Use: Control motor power consumption and ability to drive loads
    */
    m_ihold_irun.sr = tmc2300_readInt(&tmc2300, m_ihold_irun.address);
    m_ihold_irun.ihold = 0; // Standstill current
    m_ihold_irun.irun = 2; // Motor run current
    m_ihold_irun.iholddelay = 2; // Number of clock cycles for motor power down after standstill detected
    tmc2300_writeInt(&tmc2300, m_ihold_irun.address, m_ihold_irun.sr);


    /* Register: VACTUAL
    * What: Motor velocity value in +-(2^23)-1 [μsteps / t]
    * Use: Set to 0 for STEP/DIR operation, otherwise provide value for UART control to set motor velocity.
    */
    m_vactual.sr = 10000U;
    tmc2300_writeInt(&tmc2300, m_vactual.address, m_vactual.sr);

    /* Register: CHOPCONF
    * What: Generic chopper algorithm configuration 
    * Use: Configuring output stage power management and step resolution parameters
    */
    m_chopconf.sr = tmc2300_readInt(&tmc2300, m_chopconf.address);
    m_chopconf.enabledrv = 1; // Driver enable (0=disable, 1=enable)
    m_chopconf.tbl = 2; // Comparator blank time in clock-counts
    m_chopconf.mres = 3; // Microstep setting (0=256 μsteps)
    m_chopconf.intpol = 1; // Interpolation to 256 μsteps
    m_chopconf.dedge = 0; // Enable double edge step pulses
    m_chopconf.diss2g = 0; // Short to GND protection (0=enable, 1=disable)
    m_chopconf.diss2vs = 0; // Low side short protection (0=enable, 1=disable)
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
    m_pwmconf.pwm_ofs = 36; // User defined PWM amplitude offset
    m_pwmconf.pwm_grad = 16; // Velocity dependent gradient for PWM amplitude
    m_pwmconf.pwm_freq = 0; // PWM frequency selection
    m_pwmconf.pwm_autoscale = 0; // PWM automatic amplitude scaling
    m_pwmconf.pwm_autograd = 0; // PWM automatic gradient adaptation
    m_pwmconf.freewheel = 1; // Standstill option when motor current setting is 0
    m_pwmconf.pwm_reg = 4; // Regulation loop gradient
    m_pwmconf.pwm_lim = 12; // PWM automatic scale amplitude limit when switching on
    tmc2300_writeInt(&tmc2300, m_pwmconf.address, m_pwmconf.sr);
}


void TMCControl::move(uint32_t velocity)
{
    m_vactual.sr = velocity;
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

void TMCControl::enableUartPins(bool enablePins)
{
    if (enablePins && !m_uart_pins_enabled)
    {
        Utils::log_info("UART pins enable");
        // Set the TX and RX pins by using the function select on the GPIO
        // Set datasheet for more information on function select
        gpio_init(UART_TX_PIN);
        gpio_set_dir(UART_TX_PIN, GPIO_OUT);
        gpio_init(UART_RX_PIN);
        gpio_set_dir(UART_RX_PIN, GPIO_IN);
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

        m_uart_pins_enabled = true;
    }
    else if (!enablePins)
    {
        Utils::log_info("UART pins disable");
        // Set the UART pins as standard IO's
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);
        gpio_set_function(UART_RX_PIN, GPIO_FUNC_SIO);

        // Disable any pull-ups/downs on the IO pins
        gpio_disable_pulls(UART_TX_PIN);
        gpio_disable_pulls(UART_RX_PIN);

        // Set the UART pins as inputs
        gpio_set_input_enabled(UART_TX_PIN, true);
        gpio_set_input_enabled(UART_RX_PIN, true);

        m_uart_pins_enabled = false;
    }

    // TODO: Figure out if actually needed...
    sleep_ms(10);
}

void TMCControl::processJob(uint32_t tick_count)
{
    tmc2300_periodicJob(&tmc2300, tick_count);
}

extern "C" void tmc2300_readWriteArray(uint8_t channel,
                                       uint8_t *data,
                                       size_t writeLength,
                                       size_t readLength)
{
    // Write data buffer
    uart_write_blocking(UART_ID, data, writeLength);
    // Read out echo'd data to a nullptr (don't care)
    // UART is using a fifo so need to clear it for every read event
    uart_read_blocking(UART_ID, nullptr, writeLength);

    // If no reply data is expected abort here
    if (readLength == 0) return;

    // Read the reply data
    uart_read_blocking(UART_ID, data, readLength);
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
    gpio_put(PIN_TMC_N_STANDBY, enable_standby ? 0 : 1);

    // Update the APIs internal standby state
    tmc2300_setStandby(&tmc2300, enable_standby ? 1 : 0);
}

void TMCControl::enableDriver(bool enable_driver)
{
    bool _enable_driver = (bool)(driver_can_be_enabled && enable_driver);
    gpio_put(PIN_TMC_ENABLE, _enable_driver ? 1 : 0);

    // TODO: Check if necessary
    sleep_ms(10);
}

