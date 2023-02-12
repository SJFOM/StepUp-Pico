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

TMC2300TypeDef* tmc2300;
ConfigurationTypeDef* tmc2300_config;
uint8_t channel = 0;

TMCControl::TMCControl() {
    m_init_success = false;
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

        // Initialize CRC calculation for TMC2300 UART datagrams
        if (!(tmc_fillCRC8Table(0x07, true, 0) == 1))
        {
            _init_routine_success &= false;
        }

        // Configure TMC2300 IC and default register states
        (void)tmc2300_init(tmc2300, channel, tmc2300_config, tmc2300_defaultRegisterResetState);

        // Set up our UART with the required speed.   
        if (!(uart_init(UART_ID, BAUD_RATE) == BAUD_RATE))
        {
            _init_routine_success &= false;
        }

        // Set the TX and RX pins by using the function select on the GPIO
        // Set datasheet for more information on function select
        (void)gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
        (void)gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);


        // Check we have established a connection with the TMC2300 by reading its serial number
        uint8_t tmc_version = tmc2300_readInt(tmc2300, TMC2300_IOIN & TMC2300_VERSION_MASK) >> TMC2300_VERSION_SHIFT;
        if (!(tmc_version >= 0x40))
        {
            _init_routine_success &= false;
        }

        // Complete checks and store init routine success value
        m_init_success = _init_routine_success;
    }

    return m_init_success;
}

void TMCControl::deinit()
{
    // Reset the tmc2300 to its default values and state
    tmc2300_reset(tmc2300);

    // De-initialise the uart peripheral
    uart_deinit(UART_ID);

    // Set the UART pins as standard IO's
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_SIO);

    // Disable any pull-ups/downs on the IO pins
    gpio_disable_pulls(UART_TX_PIN);
    gpio_disable_pulls(UART_RX_PIN);

    // Set the UART pins as inputs
    gpio_set_input_enabled(UART_TX_PIN, true);
    gpio_set_input_enabled(UART_RX_PIN, true);

    m_init_success = false;
}

void TMCControl::processJob()
{
    tmc2300_periodicJob(tmc2300, channel);
}

extern "C"
{
    void tmc2300_readWriteArray(uint8_t channel, uint8_t* data, size_t writeLength, size_t readLength)
    {
        // TMCSerial.write(data, writeLength);
        uart_write_blocking(UART_ID, data, writeLength);

        // Wait for the written data to be received and discard it.
        // This is the echo of our tx caused by using a single wire UART.
        // while (TMCSerial.available() < writeLength)
        //     ;
        // TMCSerial.readBytes(data, writeLength);
        uart_read_blocking(UART_ID, data, writeLength);

        // If no reply data is expected abort here
        if (readLength == 0) return;

        // Wait for the reply data to be received
        while (uart_is_readable(UART_ID) < readLength)
        {
            ;
        }

        // Read the reply data
        // TMCSerial.readBytes(data, readLength);
        uart_read_blocking(UART_ID, data, readLength);

    }

    uint8_t tmc2300_CRC8(uint8_t* data, size_t length)
    {
        // tmc_CRC8 is a generic library method so will need to be mapped 
        // to the tmc2300_CRC8 extern method.
        return tmc_CRC8(data, length, 0);
    }

} // extern "C"
