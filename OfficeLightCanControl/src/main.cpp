#include <Arduino.h>

// Definições de hardware
const int LED_PIN = 1;      // Pino do LED (PWM)
const int LDR_PIN = A0;     // Pino do LDR (Leitura ADC)
const float Vcc = 3.3;      // Tensão de referência do ADC
const float R_fixed = 10000; // Resistor fixo (10kΩ)

// Coeficientes iniciais para conversão LDR → LUX (ajustáveis)
float m = -0.8;
float b = 5.976;

const int DAC_RANGE = 4096;  // Resolução do PWM (12 bits)
int counter = 0;  // Contador para variação do PWM

// Função que converte tensão medida no ADC para LUX
float voltageToLux(int adc_value) {
    float V_adc = (adc_value * Vcc) / DAC_RANGE;  // Conversão ADC → Volts
    float lux = 0;

    if (V_adc > 0) { // Evita divisão por zero
        float R_ldr = R_fixed * ((Vcc / V_adc) - 1); // Calcula resistência do LDR
        if (R_ldr > 0) {
            float logLux = (log10(R_ldr) - b) / m;  // Usa equação log-log
            lux = pow(10, logLux);  // Converte log(LUX) para LUX real
        }
    }
    return lux;
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12); // Configura ADC para 12 bits (0-4095)
    analogWriteFreq(60000); // Frequência PWM em 60KHz
    analogWriteRange(DAC_RANGE); // Define o range do PWM

    Serial.println("Starting calibration... Adjust LED brightness and observe LUX values.");
}

void loop() {
    // Define o PWM do LED
    analogWrite(LED_PIN, counter); 
    delay(5); // Pequeno tempo para estabilizar

    // Lê o ADC e converte para LUX
    int read_adc = analogRead(LDR_PIN);
    float lux = voltageToLux(read_adc);

    // Ajusta o PWM para testar diferentes intensidades
    counter = counter + 1;
    if (counter > DAC_RANGE) counter = 0;

    // Exibe os valores no Serial Monitor
    Serial.printf("ADC:%d, Voltage:%.2fV, LUX:%.2f, PWM:%d\n", read_adc, (read_adc * Vcc) / DAC_RANGE, lux, counter);
    
    delay(100); // Intervalo para coleta de dados
}
