#include <Arduino.h>

// Hardware setup
const int LED_PIN = 1;      // LED PWM pin
const int LDR_PIN = A0;     // LDR analog input pin
const float Vcc = 3.3;      // ADC reference voltage
const float R_fixed = 10000; // Fixed resistor value (10kΩ)

// Coefficients for LUX conversion (to be calibrated)
float m = -0.8;
float b = 5.976;

const int ADC_RANGE = 4096; // ADC resolution (e.g., 12-bit = 4096)
const int DAC_RANGE = 4096; // PWM resolution (12-bit)
const int STEP_SIZE = DAC_RANGE / 10; // Step size for 0.1 increment
int dutyCycle = 0; // Start at 0% duty cycle

unsigned long previousMillis = 0; // Timer variable
const int interval = 2000; // 2 seconds interval

// Function to convert ADC reading to voltage
float adcToVoltage(int adc_value, int adc_range) {
    return (adc_value * Vcc) / adc_range;
}

// Function to convert voltage to LDR resistance
float voltageToResistance(float V_adc) {
    if (V_adc <= 0) return -1; // Avoid division by zero
    return R_fixed * ((Vcc / V_adc) - 1); // Voltage divider equation
}

// Function to convert ADC reading to LUX
float voltageToLux(int adc_value, int adc_range) {
    float V_adc = adcToVoltage(adc_value, adc_range);  // Convert ADC to voltage
    float R_ldr = voltageToResistance(V_adc); // Get LDR resistance

    if (R_ldr <= 0) return 0; // Avoid invalid values

    float logLux = (log10(R_ldr) - b) / m;  // Use log-log equation
    return pow(10, logLux);  // Convert log(LUX) to real LUX value
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12); // Set ADC resolution to 12-bit
    analogWriteFreq(60000); // Set PWM frequency to 60 kHz
    analogWriteRange(DAC_RANGE); // Set full-range PWM output

}

void loop() {
    unsigned long currentMillis = millis(); // Get current time

    if (currentMillis - previousMillis >= interval) { // Check if 2 seconds passed
        previousMillis = currentMillis; // Reset timer

        // Set LED brightness (PWM duty cycle)
        analogWrite(LED_PIN, dutyCycle);

        // Increase duty cycle in steps of 0.1
        dutyCycle += STEP_SIZE;
        if (dutyCycle > DAC_RANGE) { 
            dutyCycle = 0; // Reset after reaching 100%
        }
    }

    
    // Read ADC and convert to LUX
    int read_adc = analogRead(LDR_PIN);
    float voltage = adcToVoltage(read_adc, ADC_RANGE); // Convert ADC to voltage
    float resistance = voltageToResistance(voltage); // Convert voltage to resistance
    float lux = voltageToLux(read_adc, ADC_RANGE); // Convert ADC to Lux

    // Print results with the current millis value
    Serial.printf("Millis:%lu, PWM:%.1f, ADC:%d, Voltage:%.2fV, Resistance:%.2fohm, LUX:%.2f\n",
                currentMillis, dutyCycle / (float)DAC_RANGE, read_adc, voltage, resistance, lux);

}
