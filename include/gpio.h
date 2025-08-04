#ifndef GPIO_H
#define GPIO_H

#define FILTER_SIZE 10

#include <Arduino.h>
#include <pins.h>
#include <global.h>
#include <neotimer.h>

class gpio
{
public:
    gpio();
    void initialize();
    void handler();
    void setLED(uint8_t pin, bool state, bool blink = false);
    void setLineFault(bool state);
    void setBatteryFault(bool state);
    bool getMuxStatus();
    bool getChargerStatus();

    int getBatteryPercent();
    void updateBatteryStatus();

    void setChargerEnable(bool enable);

private:
    void convertADCValues();
    Neotimer *adc_timer;   // Timer for ADC readings
    Neotimer *blink_timer; // Timer for LED blinking
    uint8_t LED_PINS[7];
    bool ledBlink[7];  // Array to track LED blinking state
    bool ledState[7];  // Array to track LED states
    bool ledOutput[7]; // Array to track LED output states
    int ledPWM[7];     // Array to track LED PWM values

    // Rolling buffer and accumulator for battery and line voltages
    float battery_samples[FILTER_SIZE] = {0};
    float line_samples[FILTER_SIZE] = {0};
    int sample_index = 0;
    float battery_sum = 0;
    float line_sum = 0;

    float V_EMPTY = 11800.0;
    float V_FULL = 12600.0;
};

#endif // GPIO_H