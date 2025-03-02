void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        dutyCycle += STEP_SIZE;
        if (dutyCycle > DAC_RANGE) {
            dutyCycle = 0;
            analogWrite(LED_PIN, dutyCycle);
            delay (3000);
        }
        analogWrite(LED_PIN, dutyCycle);
    }

    // Call calculateAllValues and get the results
    auto [adcValue,voltage, resistance, lux] = luxMeter.calculateAllValues();


    Serial.printf("%lu, %.1f, %d, %.2fV, %.2f, %.2f\n",
                  currentMillis, dutyCycle / (float)DAC_RANGE, adcValue, voltage, resistance, lux);

    delay(50);
}