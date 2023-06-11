#ifndef PIN_DEFINITIONS_H_
#define PIN_DEFINITIONS_H_

/************************/
/* TMC2300 pins - START */
/************************/

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define TMC_UART_TX_PIN (4U)
#define TMC_UART_RX_PIN (5U)

// Motor control pins
#define TMC_PIN_ENABLE    (14U)
#define TMC_PIN_N_STANDBY (15U)

/**********************/
/* TMC2300 pins - END */
/**********************/

/*************************/
/* Joystick pins - START */
/*************************/

// Joystick ADC pins
#define JOYSTICK_ADC_PIN_X            (26U)
#define JOYSTICK_ADC_CHANNEL_X        (0U)
#define JOYSTICK_ADC_PIN_Y            (27U)
#define JOYSTICK_ADC_CHANNEL_Y        (1U)
#define JOYSTICK_ADC_ROUND_ROBIN_MASK (0x03)

// Joystick Button pin
#define JOYSTICK_BUTTON_PIN (22U)

/***********************/
/* Joystick pins - END */
/***********************/

#endif  // PIN_DEFINITIONS_H_