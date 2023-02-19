#ifndef TMC_CONTROL_H_
#define TMC_CONTROL_H_

// pico-sdk
#include "pico/stdlib.h" // Includes `hardware_gpio.h`

// Common
#include "../../../Common/utils.h"

// TMC-API
extern "C"
{
#include "../../../Interfaces/ControlInterface.h"
#include "../../../Libraries/TMC_API/ic/TMC2300.h"
#include "../../../Libraries/TMC_API/helpers/CRC.h"
#include "../../../Libraries/TMC_API/helpers/Config.h"
}

// printf can default to using uart0 so use uart1 instead
#define UART_ID uart1
#define BAUD_RATE ((uint)9600)

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 4U
#define UART_RX_PIN 5U

class TMCControl: ControlInterface
{
public:
    TMCControl();
    ~TMCControl();
    bool init();
    void deinit();
    void processJob();
protected:
private:
    bool m_init_success;
};

#endif // TMC_CONTROL_H_