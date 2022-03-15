#ifndef PINS_H 
#define PINS_H

// motor pins
#define FL_PWM PA3
#define FL_DIG PB3

#define FR_PWM PA1
#define FR_DIG PB5

#define BL_PWM PA2
#define BL_DIG PB4

#define BR_PWM PA0
#define BR_DIG PB6

// mux pins
#define mux_A1 PA8
#define mux_A2 PA11
#define mux_A3 PA12
#define mux_A4 PB15

#define mux_B1 PA5
#define mux_B2 PA6
#define mux_B3 PA7
#define mux_B4 PB14

#define SIG_A PA4
#define SIG_B PB0

// tof pins
#define SHUT_1 PA12
#define SHUT_2 PA0
#define SHUT_3 PA4
#define SHUT_4 PA6

#define INT_1 PA11
#define INT_2 PA1
#define INT_3 PA5
#define INT_4 PA7

#define LAYER4_SDA PB11
#define LAYER4_SCL PB10

// misc pins
#define DRIBBLER_PIN 18
#define KICKER_PIN 12
#define LIGHT_GATE_PIN A9
#define STM32_LED PB1
#define BOARD_LED_1 8
#define BOARD_LED_2 9

#endif 