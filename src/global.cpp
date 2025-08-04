#include <global.h>

HardwareSerial DEBUG_SERIAL(DEBUG_RX, DEBUG_TX);

volatile uint16_t adc_buffer[3] = {0, 0, 0}; // Buffer for ADC readings
int battery_voltage = 0;                     // Variable to store battery voltage
int line_voltage = 0;                        // Variable to store line voltage
int battery_stddev = 0;                      // Variable to store battery voltage standard deviation
int line_stddev = 0;                         // Variable to store line voltage standard deviation