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
uint8_t channel = 0;
uint8_t slave_address = 3;
volatile bool callback_complete = false;

void callback(TMC2300TypeDef* tmc2300, ConfigState cfg_state)
{
    printf("callback...\n");

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
        printf("\ncallback - config complete!\n");
        // Configuration restore complete
        callback_complete = true;
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

        gpio_init(PIN_TMC_STANDBY);
        gpio_set_dir(PIN_TMC_STANDBY, GPIO_OUT);


        // Initialize CRC calculation for TMC2300 UART datagrams
        if (!(tmc_fillCRC8Table(0x07, true, 0) == 1))
        {
            Utils::log_debug("Error: Fill CRC");
            _init_routine_success &= false;
        }

        // Configure TMC2300 IC and default register states
        tmc2300_init(&tmc2300, channel, &tmc2300_config, tmc2300_defaultRegisterResetState);

        tmc2300_setSlaveAddress(&tmc2300, slave_address);

        tmc2300_setCallback(&tmc2300, callback);

        // Set up our UART with the required speed.   
        enableUartPins(true);
        uint _baud = uart_init(UART_ID, BAUD_RATE);
        if (!(_baud == BAUD_RATE))
        {
            Utils::log_debug("Error: UART init");
            Utils::log_debug(std::to_string(_baud));
            _init_routine_success &= false;
        }

        /*
        tmc2300_setStandby(&tmc2300, 0);
        uint8_t gS = tmc2300_getStandby(&tmc2300);
        printf("get standby: %d\n", gS);

        int count = 0;
        while (!callback_complete)
        {
            // TODO: Potentially caught in this loop forever
            printf("count: %d\n", count++);
            tmc2300_periodicJob(&tmc2300, count);
            sleep_ms(10);
        }
        callback_complete = false;


        printf("\n\nNext step\n\n");

        */

        setStandby(false);
        enableDriver(false);

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
    if (!(tmc_version >= 0x40))
    {
        Utils::log_debug("Error: TMC version");
    }

    printf("TMC version: 0x%02x\n", tmc_version);
    Utils::log_debug("TMC version: ");
    Utils::log_debug(std::to_string(tmc_version));
    return tmc_version;
}

void TMCControl::enableUartPins(bool enablePins)
{
    if (enablePins && !m_uart_pins_enabled)
    {
        printf("UART pins enable\n");
        // Set the TX and RX pins by using the function select on the GPIO
        // Set datasheet for more information on function select
        gpio_init(UART_TX_PIN);
        gpio_set_dir(UART_TX_PIN, GPIO_OUT);
        gpio_init(UART_RX_PIN);
        gpio_set_dir(UART_RX_PIN, GPIO_IN);
        (void)gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
        (void)gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

        m_uart_pins_enabled = true;
    }
    else if (!enablePins)
    {
        printf("UART pins disable\n");
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
    sleep_ms(500);
}

void TMCControl::processJob(uint32_t tick_count)
{
    tmc2300_periodicJob(&tmc2300, tick_count);
}

extern "C" void tmc2300_readWriteArray(uint8_t channel, uint8_t * data, size_t writeLength, size_t readLength)
{
    printf("rw array - channel: %d - read: %d - write: %d\n", channel, readLength, writeLength);
    // TMCSerial.write(data, writeLength);
    uart_write_blocking(UART_ID, data, writeLength);
    printf("RWA - 1\n");

    // Wait for the written data to be received and discard it.
    // This is the echo of our tx caused by using a single wire UART.
    // while (TMCSerial.available() < writeLength)
    //     ;
    // TMCSerial.readBytes(data, writeLength);
    // uart_read_blocking(UART_ID, data, writeLength);
    printf("RWA - 2\n");

    // If no reply data is expected abort here
    if (readLength == 0) return;
    printf("RWA - 3\n");

    // Wait for the reply data to be received
    // TODO: Potentially remove this?
    // while (uart_is_readable(UART_ID) < readLength)
    // {
    //     ;
    // }

    // Read the reply data
    // TMCSerial.readBytes(data, readLength);
    for (int i = 0; i < writeLength; i++)
    {
        uart_read_blocking(UART_ID, &data[i], 1);
        printf("read - %d - 0x%02x\n", i, data[i]);
    }
    printf("\n\n");
    for (int i = 0; i < readLength; i++)
    {
        uart_read_blocking(UART_ID, &data[i], 1);
        printf("read - %d - 0x%02x\n", i, data[i]);
    }
    // uart_read_blocking(UART_ID, data, readLength);
    printf("RWA - 4\n");

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
    // UART_setEnabled(TMC2300_UARTChannel, !enableStandby);
    enableUartPins(!enableStandby);

    if (enableStandby)
    {
        // Just entered standby -> disable the driver
        enableDriver(false);
    }
    // Update the APIs internal standby state
    tmc2300_setStandby(&tmc2300, enableStandby ? 1 : 0);
}

void TMCControl::enableDriver(bool enableDriver)
{
    gpio_put(PIN_TMC_ENABLE, enableDriver ? 1 : 0);

    // TODO: Check if necessary
    sleep_ms(500);
}

void TMCControl::testFunction()
{
    tmc2300_reset(&tmc2300);
}
