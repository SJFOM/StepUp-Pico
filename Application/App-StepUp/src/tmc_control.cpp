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

#include "../include/tmc_control.h"


static TMC2300TypeDef tmc2300;
static ConfigurationTypeDef tmc2300_config;

static volatile bool is_callback_complete = false;


void callback(TMC2300TypeDef* tmc2300, ConfigState cfg_state)
{
    UNUSED(tmc2300);

    if (cfg_state == CONFIG_RESET)
    {
        // Configuration reset completed
        // Change hardware preset registers here

        // Lower the default run and standstill currents
        tmc2300_writeInt(tmc2300, TMC2300_IHOLD_IRUN, 0x00010402);
    }
    else
    {
        // Configuration restore complete
        is_callback_complete = true;
        // The driver may only be enabled once the configuration is done
        // enableDriver(DRIVER_USE_GLOBAL_ENABLE);
    }
}

TMCControl::TMCControl() {
    m_init_success = false;
    m_uart_pins_enabled = false;
}
TMCControl::~TMCControl() {
    deinit();
}

bool TMCControl::init() {

    // Avoid multiple calls to this method performing the same configuration steps
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
        tmc2300_init(&tmc2300, TMC_UART_CHANNEL, &tmc2300_config, tmc2300_defaultRegisterResetState);

        tmc2300_setSlaveAddress(&tmc2300, TMC_UART_SLAVE_ADDRESS);

        tmc2300_setCallback(&tmc2300, callback);

        // Set up our UART with the required speed.   
        enableUartPins(true);
        uint _baud = uart_init(UART_ID, BAUD_RATE);
        if (!(_baud >= BAUD_RATE))
        {
            Utils::log_warn("UART init");
            Utils::log_warn((string)"Actual BAUD: " + std::to_string(_baud));
            _init_routine_success &= false;
        }

        setStandby(false);
        enableDriver(false);
        unsigned callback_safe_count = 0;
        while (!is_callback_complete)
        {
            if (callback_safe_count++ > 100)
            {
                // Typical count is <10, we should have high conviction here that something is wrong.
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

uint8_t TMCControl::getChipID()
{
    // Check we have established a connection with the TMC2300 by reading its serial number
    int32_t tmc_version = tmc2300_readInt(&tmc2300, TMC2300_IOIN);
    tmc_version = ((tmc_version & TMC2300_VERSION_MASK) >> TMC2300_VERSION_SHIFT);
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

extern "C" void tmc2300_readWriteArray(uint8_t channel, uint8_t * data, size_t writeLength, size_t readLength)
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

extern "C" uint8_t tmc2300_CRC8(uint8_t * data, size_t length)
{
    // tmc_CRC8 is a generic library method so will need to be mapped 
    // to the tmc2300_CRC8 extern method.
    return tmc_CRC8(data, length, 0);
}

void TMCControl::setStandby(bool enableStandby)
{
    // En/disable the UART pins depending on standby state
    enableUartPins(!enableStandby);

    if (enableStandby)
    {
        // Just entered standby -> disable the driver
        enableDriver(false);
    }

    // Set the Standby pin state - after enable so we retain control over driver
    gpio_put(PIN_TMC_N_STANDBY, enableStandby ? 0 : 1);

    // Update the APIs internal standby state
    tmc2300_setStandby(&tmc2300, enableStandby ? 1 : 0);
}

void TMCControl::enableDriver(bool enableDriver)
{
    gpio_put(PIN_TMC_ENABLE, enableDriver ? 1 : 0);

    // TODO: Check if necessary
    sleep_ms(10);
}

void TMCControl::testFunction()
{
    tmc2300_reset(&tmc2300);
}
