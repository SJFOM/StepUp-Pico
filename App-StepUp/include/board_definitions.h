#ifndef BOARD_DEFINITION_H_
#define BOARD_DEFINITION_H_

/******************************/
/* Power control pins - START */
/******************************/
#define MCU_PWR_CTRL_PIN (7U)
#define MCU_PWR_BTN_PIN  (8U)

/****************************/
/* Power control pins - END */
/****************************/

/***********************/
/* Status pins - START */
/***********************/

#if PCB_REVISION == 1U
#    define LED_PIN_RED   (2U)  // PWM1_A
#    define LED_PIN_GREEN (3U)  // PWM1_B
#    define LED_PIN_BLUE  (5U)  // PWM2_B
#elif PCB_REVISION == 2U
#    define LED_PIN_RED   (3U)  // PWM1_B
#    define LED_PIN_GREEN (6U)  // Unsure if assigned to any PWM...
#    define LED_PIN_BLUE  (5U)  // PWM2_B
#else
#    error "No valid PCB version found!!"
#endif

#define BUZZER_PIN (16U)  // PWM0_A

/*********************/
/* Status pins - END */
/*********************/

/************************/
/* TMC2300 pins - START */
/************************/
#define R_SENSE (0.15f)

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define TMC_PIN_UART_TX (25U)
#define TMC_PIN_UART_RX (24U)

// Motor control pins
#define TMC_PIN_ENABLE    (20U)
#define TMC_PIN_N_STANDBY (21U)

// Diagnostics pins
#define TMC_PIN_DIAG (19U)

#define TMC_PIN_BOOST_EN (18U)

// STEP/DIR pins
#define TMC_PIN_STEP (23U)  // PWM3_B
#define TMC_PIN_DIR  (22U)

/**********************/
/* TMC2300 pins - END */
/**********************/

/********************************/
/* Voltage monitor pins - START */
/********************************/

#define VMOTOR_ADC_SCALING_FACTOR (6U)
#define VBAT_ADC_SCALING_FACTOR   (2U)

#define VBAT_MONITOR_ADC_PIN       (26U)
#define VBAT_MONITOR_ADC_CHANNEL   (0)
#define VMOTOR_MONITOR_ADC_PIN     (27U)
#define VMOTOR_MONITOR_ADC_CHANNEL (1U)

#define VUSB_MONITOR_PIN (4U)  // USB_VBUS_DET

/******************************/
/* Voltage monitor pins - END */
/******************************/

/*************************/
/* Joystick pins - START */
/*************************/

// Joystick ADC pins
#if PCB_REVISION == 1U
#    define JOYSTICK_ADC_PIN_X     (29U)
#    define JOYSTICK_ADC_CHANNEL_X (3U)
#    define JOYSTICK_ADC_PIN_Y     (28U)
#    define JOYSTICK_ADC_CHANNEL_Y (2U)
#elif PCB_REVISION == 2U
#    define JOYSTICK_ADC_PIN_X     (28U)
#    define JOYSTICK_ADC_CHANNEL_X (2U)
#    define JOYSTICK_ADC_PIN_Y     (29U)
#    define JOYSTICK_ADC_CHANNEL_Y (3U)
#else
#    error "No valid PCB version found!!"
#endif
#define JOYSTICK_ADC_ROUND_ROBIN_MASK (0x03)

// Joystick Button pin
#define JOYSTICK_BUTTON_PIN (17U)

/***********************/
/* Joystick pins - END */
/***********************/

/**************************/
/* Spare I/O pins - START */
/**************************/
#if PCB_REVISION == 1U
#    define GPIO_PIN_8 (8U)
#elif PCB_REVISION == 2U
#    define GPIO_PIN_2 (2U)
#else
#    error "No valid PCB version found!!"
#endif

#define GPIO_PIN_9  (9U)
#define GPIO_PIN_10 (10U)
#define GPIO_PIN_11 (11U)
#define GPIO_PIN_12 (12U)
#define GPIO_PIN_13 (13U)
#define GPIO_PIN_14 (14U)
#define GPIO_PIN_15 (15U)

/************************/
/* Spare I/O pins - END */
/************************/

#endif  // BOARD_DEFINITION_H_