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
#define BAUD_RATE ((uint)115200)

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 4U
#define UART_RX_PIN 5U

// Motor control pins
#define PIN_TMC_ENABLE 14U
#define PIN_TMC_N_STANDBY 15U

#define TMC_UART_SLAVE_ADDRESS (3U)

#define TMC_UART_CHANNEL (0) // Not as relevant for single IC use case


/*****************************/
/* General Registers - START */
/*****************************/
struct GCONF_t {
    constexpr static uint8_t address = TMC2300_GCONF;
    union {
        uint8_t sr;
        struct {
            bool :1,
                extcap : 1,
                : 1,
                shaft : 1,
                diag_index : 1,
                diag_step : 1,
                multistep_filt : 1,
                test_mode : 1;
        };
    };
};

struct GSTAT_t {
    constexpr static uint8_t address = TMC2300_GSTAT;
    union {
        uint8_t sr : 3;
        struct {
            bool  reset : 1,
                drv_err : 1,
                u3v5 : 1;
        };
    };
};


struct IOIN_t {
    constexpr static uint8_t address = TMC2300_IOIN;
    union {
        uint32_t sr;
        struct {
            bool  en : 1,
                nstdby : 1,
                ad0 : 1,
                ad1 : 1,
                diag : 1,
                stepper : 1,
                pdn_uart : 1,
                mode : 1,
                step : 1,
                dir : 1,
                comp_a1a2 : 1,
                comp_b1b2 : 1;
            uint16_t : 12;
            uint8_t version : 8;
        };
    };
};

/***************************/
/* General Registers - END */
/***************************/

/**************************************/
/* Velocity Dependent Control - START */
/**************************************/
struct IHOLD_IRUN_t {
    constexpr static uint8_t address = TMC2300_IHOLD_IRUN;
    union {
        uint32_t sr : 20;
        struct {
            uint8_t ihold : 5,
                : 3,
                irun : 5,
                : 3,
                iholddelay : 4;
        };
    };
};


struct TPOWERDOWN_t {
    constexpr static uint8_t address = TMC2300_TPOWERDOWN;
    uint8_t sr : 8;
};

struct TSTEP_t {
    constexpr static uint8_t address = TMC2300_TSTEP;
    uint32_t sr : 20;
};

struct VACTUAL_t {
    constexpr static uint8_t address = TMC2300_VACTUAL;
    uint32_t sr : 24;
};

/************************************/
/* Velocity Dependent Control - END */
/************************************/

/******************************/
/* StallGuard Control - START */
/******************************/
struct TCOOLTHRS_t {
    constexpr static uint8_t address = TMC2300_TCOOLTHRS;
    uint16_t sr : 10;
};

struct SGTHRS_t {
    constexpr static uint8_t address = TMC2300_SGTHRS;
    uint8_t sr : 8;
};

struct SGVALUE_t {
    constexpr static uint8_t address = TMC2300_SG_VALUE;
    uint16_t sr : 10;
};

struct COOLCONF_t {
    constexpr static uint8_t address = TMC2300_COOLCONF;
    union {
        uint16_t sr : 16;
        struct {
            uint8_t semin : 4,
                : 1,
                seup : 2,
                : 1,
                semax : 4,
                : 1,
                sedn : 2;
            bool    seimin : 1;
        };
    };
};

/****************************/
/* StallGuard Control - END */
/****************************/

// TODO: Section 5.4 - Sequencer Registers
// TODO: Section 5.5 - Chopper Control Registers


class TMCControl: ControlInterface
{
public:
    TMCControl();
    ~TMCControl();
    bool init();
    void deinit();
    void processJob(uint32_t tick_count);
    void enableUartPins(bool enablePins);
    void setStandby(bool enableStandby);
    void enableDriver(bool enableDriver);
    uint8_t getChipID();
    void testFunction();
protected:
private:
    bool m_init_success, m_uart_pins_enabled;
};

#endif // TMC_CONTROL_H_