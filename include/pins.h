#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// ADC
#define ADC_BATTERY PA0
#define ADC_LINE PA1

// LEDS
#define LED_LOW PA4
#define LED_MED PA6
#define LED_HIGH PA7
#define LED_FULL PB0
#define LED_CHGR PB6
#define LED_LINE PB8
#define LED_BAT PB9

// OUTPUTS
#define ERROR_BAT PA2
#define ERROR_LINE PA3

// CHARGER
#define STATUS_CHGR PA5
#define CHGR_EN PA12

// MUX
#define STATUS_MUX PC15

// USART
#define DEBUG_TX_RX PA9_R

#endif
