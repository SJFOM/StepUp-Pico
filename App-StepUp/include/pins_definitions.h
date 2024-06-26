#ifndef PIN_DEFINITIONS_H_
#define PIN_DEFINITIONS_H_

/***************************/
/* LED Status pins - START */
/***************************/
#define LED_PIN_RED   (2U)
#define LED_PIN_GREEN (3U)
#define LED_PIN_BLUE  (5U)

/*************************/
/* LED Status pins - END */
/*************************/

/************************/
/* TMC2300 pins - START */
/************************/

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define TMC_UART_TX_PIN (25U)
#define TMC_UART_RX_PIN (24U)

// Motor control pins
#define TMC_ENABLE_PIN    (20U)
#define TMC_N_STANDBY_PIN (21U)

// Diagnostics pins
#define TMC_DIAG_PIN (19U)

#define TMC_BOOST_EN (18U)

/**********************/
/* TMC2300 pins - END */
/**********************/

/*************************/
/* Joystick pins - START */
/*************************/

// Joystick ADC pins
#define JOYSTICK_ADC_PIN_X            (29U)
#define JOYSTICK_ADC_CHANNEL_X        (3U)
#define JOYSTICK_ADC_PIN_Y            (28U)
#define JOYSTICK_ADC_CHANNEL_Y        (2U)
#define JOYSTICK_ADC_ROUND_ROBIN_MASK (0x03)

// Joystick Button pin
#define JOYSTICK_BUTTON_PIN (17U)

/***********************/
/* Joystick pins - END */
/***********************/

#endif  // PIN_DEFINITIONS_H_