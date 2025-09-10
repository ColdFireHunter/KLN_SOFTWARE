#include <gpio.h>

gpio::gpio()
{
    adc_timer = new Neotimer(25);    // Timer for ADC readings
    blink_timer = new Neotimer(500); // Timer for LED blinking
    LED_PINS[0] = LED_LOW;
    LED_PINS[1] = LED_MED;
    LED_PINS[2] = LED_HIGH;
    LED_PINS[3] = LED_FULL;
    LED_PINS[4] = LED_CHGR;
    LED_PINS[5] = LED_BAT;
    LED_PINS[6] = LED_LINE;

    ledPWM[0] = LED_LOW_PWM_VALUE;
    ledPWM[1] = LED_MED_PWM_VALUE;
    ledPWM[2] = LED_HIGH_PWM_VALUE;
    ledPWM[3] = LED_FULL_PWM_VALUE;
    ledPWM[4] = LED_CHGR_PWM_VALUE;
    ledPWM[5] = LED_BAT_PWM_VALUE;
    ledPWM[6] = LED_LINE_PWM_VALUE;

    for (int i = 0; i < 7; i++)
    {
        ledBlink[i] = false;  // Initialize LED blinking state
        ledState[i] = false;  // Initialize LED states
        ledOutput[i] = false; // Initialize LED output states
    }
}

void gpio::initialize()
{
    // Configure all LEDs as outputs, start off
    for (int i = 0; i < 7; ++i)
    {
        pinMode(LED_PINS[i], OUTPUT);
        digitalWrite(LED_PINS[i], LOW);
    }
    // Fault outputs
    pinMode(ERROR_BAT, OUTPUT);
    digitalWrite(ERROR_BAT, LOW);
    pinMode(ERROR_LINE, OUTPUT);
    digitalWrite(ERROR_LINE, LOW);

    // Status inputs
    pinMode(STATUS_CHGR, INPUT);
    pinMode(STATUS_MUX, INPUT);

    pinMode(CHGR_EN, OUTPUT);
    digitalWrite(CHGR_EN, LOW); // Disable charger by default
}

void gpio::handler()
{
    convertADCValues();
    updateBatteryStatus();

    // Blink any LEDs in blink mode
    if (blink_timer->repeat())
    {
        for (int i = 0; i < 7; ++i)
        {
            if (ledBlink[i])
            {
                // toggle
                ledOutput[i] = !ledOutput[i];
                analogWrite(LED_PINS[i],
                            ledOutput[i] ? ledPWM[i] : 0);
            }
        }
    }
}

void gpio::setLED(uint8_t pin, bool state, bool blink)
{
    // 1) figure out if this is one of our managed LEDs
    int idx = -1;
    for (int i = 0; i < 7; ++i)
    {
        if (LED_PINS[i] == pin)
        {
            idx = i;
            break;
        }
    }

    // 2) non-managed pins: just drive them directly every time
    if (idx < 0)
    {
        analogWrite(pin, state ? ledPWM[0] : 0);
        return;
    }

    // 3) if nothing changed (same state & same blink flag), do nothing
    if (ledState[idx] == state && ledBlink[idx] == blink)
    {
        return;
    }

    // 4) record the new mode
    ledState[idx] = state;
    ledBlink[idx] = blink;

    // 5) update output once for this change
    ledOutput[idx] = state;
    if (!blink)
    {
        // static on/off, single write
        analogWrite(LED_PINS[idx], state ? ledPWM[idx] : 0);
    }
    else
    {
        // start blinking: one initial write, then handler()’s blinkTimer does the toggles
        analogWrite(LED_PINS[idx], state ? ledPWM[idx] : 0);
    }
}

void gpio::setLineFault(bool state)
{
    digitalWrite(ERROR_LINE, state ? HIGH : LOW);
}

void gpio::setBatteryFault(bool state)
{
    digitalWrite(ERROR_BAT, state ? HIGH : LOW);
}

bool gpio::getMuxStatus()
{
    // HIGH = one state, LOW = the other
    return digitalRead(STATUS_MUX) == HIGH;
}

bool gpio::getChargerStatus()
{
    return digitalRead(STATUS_CHGR) == LOW;
}

void gpio::convertADCValues()
{
    if (adc_timer->repeat())
    {
        // Calculate reference voltage
        int vref = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_buffer[2], ADC_RESOLUTION_12B);

        // Read raw voltages
        float new_battery = __HAL_ADC_CALC_DATA_TO_VOLTAGE(adc_buffer[0], vref, ADC_RESOLUTION_12B) * 6.369426751592357;
        float new_line = __HAL_ADC_CALC_DATA_TO_VOLTAGE(adc_buffer[1], vref, ADC_RESOLUTION_12B) * 16.39344262295082;

        // Subtract oldest sample from sums
        battery_sum -= battery_samples[sample_index];
        line_sum -= line_samples[sample_index];

        // Store new samples in buffer
        battery_samples[sample_index] = new_battery;
        line_samples[sample_index] = new_line;

        // Add new samples to sums
        battery_sum += new_battery;
        line_sum += new_line;

        // Move to next index (wrap around)
        sample_index = (sample_index + 1) % FILTER_SIZE;

        // Compute rolling average over FILTER_SIZE samples
        battery_voltage = battery_sum / FILTER_SIZE;
        line_voltage = line_sum / FILTER_SIZE;

        // calculate std dev
        float battery_variance = 0.0f;
        float line_variance = 0.0f;
        for (int i = 0; i < FILTER_SIZE; ++i)
        {
            battery_variance += pow(battery_samples[i] - battery_voltage, 2);
            line_variance += pow(line_samples[i] - line_voltage, 2);
        }

        battery_variance /= FILTER_SIZE;
        line_variance /= FILTER_SIZE;
        battery_stddev = int(round(sqrt(battery_variance)));
        line_stddev = int(round(sqrt(line_variance)));
    }
}

int gpio::getBatteryPercent()
{
    float pct = (float(battery_voltage) - V_EMPTY) / (V_FULL - V_EMPTY) * 100.0f;
    return int(constrain(pct, 0.0f, 100.0f));
}

void gpio::updateBatteryStatus()
{
    int pct = getBatteryPercent();
    if (pct >= 90.0f)
    {
        setLED(LED_LOW, true, false);
        setLED(LED_MED, true, false);
        setLED(LED_HIGH, true, false);
        setLED(LED_FULL, true, false);
    }
    else if (pct >= 60.0f)
    {
        setLED(LED_LOW, true, false);
        setLED(LED_MED, true, false);
        setLED(LED_HIGH, true, false);
        setLED(LED_FULL, false, false);
    }
    else if (pct >= 40.0f)
    {
        setLED(LED_LOW, true, false);
        setLED(LED_MED, true, false);
        setLED(LED_HIGH, false, false);
        setLED(LED_FULL, false, false);
    }
    else if (pct >= 15.0f)
    {
        setLED(LED_LOW, true, false);
        setLED(LED_MED, false, false);
        setLED(LED_HIGH, false, false);
        setLED(LED_FULL, false, false);
    }
    else
    {
        setLED(LED_LOW, true, true);
        setLED(LED_MED, false, false);
        setLED(LED_HIGH, false, false);
        setLED(LED_FULL, false, false);
    }
}
void gpio::setChargerEnable(bool enable)
{
    if (enable)
    {
        pinMode(CHGR_EN, INPUT); // Set to input to disable any drive
    }
    else
    {
        pinMode(CHGR_EN, OUTPUT);   // Set to output to control the charger
        digitalWrite(CHGR_EN, LOW); // Disable charger
    }
}