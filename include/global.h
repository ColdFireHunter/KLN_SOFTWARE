#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <pins.h>

extern HardwareSerial DEBUG_SERIAL;

#define LED_LOW_PWM_VALUE 32
#define LED_MED_PWM_VALUE 64
#define LED_HIGH_PWM_VALUE 32
#define LED_FULL_PWM_VALUE 8
#define LED_CHGR_PWM_VALUE 24
#define LED_BAT_PWM_VALUE 16
#define LED_LINE_PWM_VALUE 16

extern volatile uint16_t adc_buffer[3];
extern int battery_voltage;
extern int line_voltage;
extern int battery_stddev;
extern int line_stddev;

#define SW_VERSION "1.0" // Software version
#define HW_VERSION "A"   // Hardware version
#define SW_DATE __DATE__ // Software build date
#define SW_TIME __TIME__ // Software build time

#endif
