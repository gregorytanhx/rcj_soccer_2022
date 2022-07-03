#ifndef PINS_H 
#define PINS_H

// motor pins
#define FL_PWM PA_3
#define FL_DIG PB_3

#define FR_PWM PA_0
#define FR_DIG PB_6

#define BL_PWM PA_2
#define BL_DIG PB_4

#define BR_PWM PA_1
#define BR_DIG PB_5

// mux pins
#define mux_A1 PA_8
#define mux_A2 PA_11
#define mux_A3 PA_12
#define mux_A4 PB_15

#define mux_B1 PA_5
#define mux_B2 PA_6
#define mux_B3 PA_7
#define mux_B4 PB_14

#define sigA PA4
#define sigB PB0

// tof pins
#define SHUT_1 PA12
#define SHUT_2 PA6
#define SHUT_3 PA4
#define SHUT_4 PA0

#define INT_1 PA11
#define INT_2 PA7
#define INT_3 PA5
#define INT_4 PA1

#define LAYER4_SDA PB11
#define LAYER4_SCL PB10

// misc pins
#define DRIBBLER_PIN 18
#define KICKER_PIN 12
#define LIGHT_GATE_PIN A9
#define STM32_LED PB1
#define BOARD_LED_1 8
#define BOARD_LED_2 9
#define BT_EN_PIN 6
#define BT_RESET_PIN 2

#endif 