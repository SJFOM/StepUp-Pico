/**
 * @file tmc_control.hpp
 * @author Sam (@SJFOM)
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 * @license   MIT
 *
 */

#ifndef TMC_CONTROL_H_
#define TMC_CONTROL_H_

// pico-sdk
#include "pico/stdlib.h"  // Includes `hardware_gpio.h`

// Logging utilities
#include "PicoUtils.h"

// Control libraries
#include "ControlInterface.hpp"

// pin includes
#include "board_definitions.h"

// TMC-API
extern "C"
{
#include "../../../Libraries/TMC_API/helpers/CRC.h"
#include "../../../Libraries/TMC_API/helpers/Config.h"
#include "../../../Libraries/TMC_API/ic/TMC2300.h"
}

// printf can default to using uart0 so use uart1 instead
#define TMC_UART_ID   uart1
#define TMC_BAUD_RATE ((unsigned)460800)

#define TMC_UART_SLAVE_ADDRESS (3U)

#define TMC_UART_CHANNEL (0)  // Not as relevant for single IC use case

// Default motion profile ramp configurations
#define VELOCITY_RAMP_INCREMENT_STEPS_PER_SECOND (1000U)
#define VELOCITY_STARTING_STEPS_PER_SECOND       (10000U)

// Rate at which we decrement the velocity if we reach Stallgaurd threshold
// limits
#define VELOCITY_RAMP_SG_LIMIT_DECREMENT_STEPS_PER_SECOND \
    (VELOCITY_RAMP_INCREMENT_STEPS_PER_SECOND * 5U)

// Used for threshold where open-circuit flags are valid - datasheet says they
// are valid for "slow speed" movements
#define VELOCITY_SLOW_SPEED_STEPS_PER_SECOND \
    (2 * VELOCITY_STARTING_STEPS_PER_SECOND)

#define VELOCITY_MAX_STEPS_PER_SECOND (100000U)

// Run and hold current values (0..31U) scaled to 1.2A RMS
#define DEFAULT_IRUN_VALUE  (13U)
#define DEFAULT_IHOLD_VALUE (0U)

// If SG_VALUE falls below 2x SGTHRS_VALUE then a stall detection is triggered
// SG_VALUE = 0..510 (higher number, lighter loading)
// 90% loading = 510 - 0.9*510 = 0.1*510 - 51
// SGTHRS = 51/2 ~= 25
constexpr uint16_t CX_DEFAULT_SGTHRS_VALUE = 25U;

/*****************************/
/* General Registers - START */
/*****************************/
struct GCONF_t
{
    // READ + WRITE register
    constexpr static uint8_t address = TMC2300_GCONF;

    /* GCONF: Global Configuration Flags */
    union
    {
        uint8_t sr;
        struct
        {
            bool : 1, extcap : 1, : 1, shaft : 1, diag_index : 1, diag_step : 1,
                multistep_filt : 1, test_mode : 1;
        };
    };
    // Default constructor holds reset values
    GCONF_t() : extcap(0), multistep_filt(1) {}
};

struct GSTAT_t
{
    // READ + WRITE register, Clear upon WRITE
    constexpr static uint8_t address = TMC2300_GSTAT;

    /* GSTAT: Global Status Flags */
    union
    {
        uint8_t sr : 3;
        struct
        {
            bool reset : 1, drv_err : 1, u3v5 : 1;
        };
    };
};

struct IFCNT_t
{
    // READ only register
    constexpr static uint8_t address = TMC2300_IFCNT;

    /* Interface  transmission  counter.  This  register  becomes incremented
     * with each successful UART interface write access. Read  out  to  check
     * the serial transmission  for lost  data.  Read  accesses  do  not  change
     * the  content. The counter wraps around from 255 to 0.*/
    uint8_t sr;
};

struct SLAVECONF_t
{
    // WRITE only register
    constexpr static uint8_t address = TMC2300_SLAVECONF;
    union
    {
        uint8_t sr;
        struct
        {
            uint8_t : 8;
            uint8_t send_delay_bit_time : 4;
        };
    };
};

struct IOIN_t
{
    // READ only register
    constexpr static uint8_t address = TMC2300_IOIN;

    /* IOIN: INPUT - Reads the state of all input pins available */
    union
    {
        uint32_t sr;
        struct
        {
            bool en : 1, nstdby : 1, ad0 : 1, ad1 : 1, diag : 1, stepper : 1,
                pdn_uart : 1, mode : 1, step : 1, dir : 1, comp_a1a2 : 1,
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
struct IHOLD_IRUN_t
{
    // WRITE only register
    constexpr static uint8_t address = TMC2300_IHOLD_IRUN;

    /* IHOLD_IRUN: Driver Current control */

    union
    {
        uint32_t sr : 20;
        struct
        {
            // TODO: Re-order: check the following is now correct
            uint8_t ihold : 5, : 3, irun : 5, : 3, iholddelay : 4;
        };
    };

    // Default constructor holds reset values
    IHOLD_IRUN_t() : ihold(8U), irun(31U), iholddelay(1U) {}
};

struct TPOWERDOWN_t
{
    // WRITE only register
    constexpr static uint8_t address = TMC2300_TPOWERDOWN;

    /* Sets the delay time from stand-still (stst) detection to motor current
     * power down. Time range is about 0 to 5.6 seconds. */
    uint8_t sr : 8;

    // Default constructor holds reset value
    TPOWERDOWN_t() : sr(20U) {}
};

struct TSTEP_t
{
    // READ only register
    constexpr static uint8_t address = TMC2300_TSTEP;

    /* Actual measured time between two 1/256 microsteps derived from the step
     * input frequency in units of 1/fCLK. Measured value is (2^20)-1 in case of
     * overflow or stand still. */
    uint32_t sr : 20;
};

struct VACTUAL_t
{
    // WRITE only register
    constexpr static uint8_t address = TMC2300_VACTUAL;
    int32_t sr : 24;
};

/************************************/
/* Velocity Dependent Control - END */
/************************************/

/******************************/
/* StallGuard Control - START */
/******************************/
struct TCOOLTHRS_t
{
    // WRITE only register
    constexpr static uint8_t address = TMC2300_TCOOLTHRS;

    /* This is the lower threshold velocity for switching on smart energy
     * CoolStep and StallGuard feature. RANGE: 1..2^20-1. DEFAULT: 0 */
    uint32_t sr : 20;
};

struct SGTHRS_t
{
    // WRITE only register
    constexpr static uint8_t address = TMC2300_SGTHRS;

    /* Detection threshold for stall. The StallGuard value SG_VALUE becomes
     * compared to this threshold. */
    uint8_t sr : 8;
};

struct SGVALUE_t
{
    // READ only register
    constexpr static uint8_t address = TMC2300_SG_VALUE;

    /* StallGuard result. SG_RESULT becomes updated with each fullstep,
     * independent of TCOOLTHRS and SGTHRS. A higher value signals a lower motor
     * load and more torque headroom. Intended for StealthChop mode, only. Bits
     * 9 and 0 will always show 0. Scaling to 10 bit is for compatibility to
     * StallGuard2. */
    uint16_t sr : 10;
};

struct COOLCONF_t
{
    // WRITE only register
    constexpr static uint8_t address = TMC2300_COOLCONF;

    /* COOLCONF: Smart energy control CoolStep and StallGuard. DEFAULT: 0
     * (disabled)*/
    union
    {
        uint16_t sr;
        struct
        {
            /* NOTE: semin = 0, smart current control coolStep = OFF */
            // FIXME: This register WORKS if the bit order is backwards to the
            // other registers?.. Look into struct packing, maybe this isn't
            // consistent?
            bool seimin : 1;
            uint8_t sedn : 2, : 1, semax : 4, : 1, seup : 2, : 1, semin : 4;
        };
    };
};

/****************************/
/* StallGuard Control - END */
/****************************/

/*******************************/
/* Sequencer Registers - START */
/*******************************/
struct MSCNT_t
{
    // READ only register
    constexpr static uint8_t address = TMC2300_MSCNT;

    /* Microstep counter. Indicates actual position in the microstep table
     * for CUR_A. CUR_B uses an offset of 256 into the table. Reading out
     * MSCNT allows determination of the motor position within the
     * electrical wave.*/
    union
    {
        uint16_t sr : 10;
    };
};

/*****************************/
/* Sequencer Registers - END */
/*****************************/

/*************************************/
/* Chopper Control Registers - START */
/*************************************/

struct CHOPCONF_t
{
    // READ + WRITE register
    constexpr static uint8_t address = TMC2300_CHOPCONF;

    /* CHOPCONF: Chopper Configuration */
    union
    {
        uint32_t sr;
        struct
        {
            bool enabledrv : 1;
            uint16_t : 14;
            uint8_t tbl : 2, : 7, mres : 4;
            bool intpol : 1, dedge : 1, diss2g : 1, diss2vs : 1;
        };
    };
    // Default constructor holds reset value
    CHOPCONF_t() : sr(0x13008001) {}
};

struct DRV_STATUS_t
{
    // READ only register
    constexpr static uint8_t address = TMC2300_DRVSTATUS;

    constexpr static uint32_t error_bit_mask =
        0x800003FF;  // 0b1000 0000 0000 0000 0000 0011 1111 1111

    /* DRV_STATUS Driver status flags and current level read back */
    union
    {
        uint32_t sr;
        struct
        {
            bool otpw : 1, ot : 1, s2ga : 1, s2gb : 1, s2vsa : 1, s2vsb : 1,
                ola : 1, olb : 1, t120 : 1, t150 : 1;
            uint8_t : 6;
            uint8_t cs_actual : 5;
            uint16_t : 10;
            bool stst : 1;
        };
    };
};

struct PWMCONF_t
{
    // READ + WRITE register
    constexpr static uint8_t address = TMC2300_PWMCONF;

    /* PWMCONF: Voltage mode PWM StealthChop */
    union
    {
        uint32_t sr;
        struct
        {
            uint8_t pwm_ofs : 8, pwm_grad : 8, pwm_freq : 2;
            bool pwm_autoscale : 1, pwm_autograd : 1;
            uint8_t freewheel : 2, : 2, pwm_reg : 4, pwm_lim : 4;
        };
    };
    // Default constructor holds reset value
    PWMCONF_t() : sr(0xC40D1024) {}
};

struct PWM_SCALE_t
{
    // READ only register
    constexpr static uint8_t address = TMC2300_PWMSCALE;
    union
    {
        uint32_t sr;
        struct
        {
            uint8_t pwm_scale_sum : 8;
            uint8_t : 8;
            int16_t pwm_scale_auto : 9;
        };
    };
};

struct PWM_AUTO_t
{
    // READ only register
    constexpr static uint8_t address = TMC2300_PWM_AUTO;
    union
    {
        uint32_t sr;
        struct
        {
            uint8_t pwm_ofs_auto : 8;
            uint8_t : 8;
            uint16_t pwm_grad_auto : 8;
        };
    };
};

/***********************************/
/* Chopper Control Registers - END */
/***********************************/

struct TMCDiagnostics
{
    bool normal_operation = true;
    bool overheating = false;
    bool short_circuit = false;
    bool open_circuit = false;
    bool stall_detected = false;
};

struct TMCData
{
    ControllerState control_state;
    TMCDiagnostics diag;
    bool open_circuit_detected = false;
};

struct TMCOpenCircuitAlgoData
{
    const uint8_t sg_val_match_count_threshold = 10U;
    uint8_t sg_val_match_count;
    uint16_t sg_val_previous;
};

enum MotorMoveState
{
    MOTOR_IDLE = 0U,
    MOTOR_IDLE_TO_MOVING = 1U,
    MOTOR_MOVING_TO_IDLE = 2U,
    MOTOR_MOVING,
};

enum FreewheelMode
{
    FREEWHEEL_NORMAL = 0,
    FREEWHEEL_FREEWHEELING = 1,
    FREEWHEEL_LS_DRIVER_SHORT = 2,
    FREEWHEEL_HS_DRIVER_SHORT = 3,
};

enum MicrostepResolution
{
    MRES_FULL_STEP = 8,
    MRES_2_STEP = 7,
    MRES_4_STEP = 6,
    MRES_8_STEP = 5,
    MRES_16_STEP = 4,
    MRES_32_STEP = 3,
    MRES_64_STEP = 2,
    MRES_12_STEP = 1,
    MRES_256_STEP = 0,
};

enum CoolStepCurrentReduction
{
    COOLSTEP_REDUCTION_1_2 = 0,
    COOLSTEP_REDUCTION_1_4 = 1U,
};

enum ComparatorBlankTime
{
    COMPARATOR_BLANK_TIME_16_CLK_CYCLES = 0,
    COMPARATOR_BLANK_TIME_24_CLK_CYCLES = 1U,  // Default
    COMPARATOR_BLANK_TIME_32_CLK_CYCLES = 2U,
    COMPARATOR_BLANK_TIME_40_CLK_CYCLES = 3U,
};

enum StealthchopPWMFrequency
{
    PWM_FREQ_2_1024_FCLK = 0,
    PWM_FREQ_2_683_FCLK = 1U,
    PWM_FREQ_2_512_FCLK = 2U,
    PWM_FREQ_2_410_FCLK = 3U,
};

class TMCControl : public ControlInterface
{
public:
    TMCControl(float r_sense, bool coolstep_enabled = false);
    ~TMCControl();
    bool init(void);
    void deinit(void);
    void defaultConfiguration(void);
    enum ControllerState processJob(uint32_t tick_count);
    void resetMovementDynamics(void);
    void updateMovementDynamics(int32_t velocity_delta, int8_t direction);
    uint8_t getChipID(void);
    struct TMCData getTMCData();

protected:
    GCONF_t m_gconf;
    GSTAT_t m_gstat;
    IOIN_t m_ioin;
    IHOLD_IRUN_t m_ihold_irun;
    VACTUAL_t m_vactual;
    SGTHRS_t m_sgthrs;
    SGVALUE_t m_sgval;
    TCOOLTHRS_t m_tcoolthrs;
    COOLCONF_t m_coolconf;
    CHOPCONF_t m_chopconf;
    TSTEP_t m_tstep;
    DRV_STATUS_t m_drv_status;
    PWMCONF_t m_pwmconf;
    PWM_SCALE_t m_pwm_scale;

private:
    MotorMoveState m_motor_move_state;
    bool m_init_success, m_uart_pins_enabled;
    struct TMCData m_tmc;
    int32_t m_target_velocity, m_ramp_velocity;
    TMCOpenCircuitAlgoData m_open_circuit_algo_data;
    bool m_coolstep_enabled, m_peak_velocity_detected;
    float m_r_sense;

    static uint16_t convertIrunIHoldToRMSCurrentInMilliamps(uint8_t i_run_hold,
                                                            float r_sense);

    void enableTMCDiagInterrupt(bool enable_interrupt);
    void enableUartPins(bool enable_pins);
    void enablePeripheralDriver(bool enable_disable) override;
    void setStandby(bool enable_standby);
    bool isDriverEnabled(void);
    void move(int32_t velocity);
    void setMotorVelocityRegisterValue(int32_t velocity);
    void setCurrent(uint8_t i_run, uint8_t i_hold);
    void updateCurrent(uint8_t i_run_delta);
    bool isOpenCircuitDetected();
    void resetOpenCircuitDetectionAlgorithm();
    TMCDiagnostics readTMCDiagnostics();
};

#endif  // TMC_CONTROL_H_