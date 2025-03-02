// luxmeter.cpp
#include <driver.h>

Driver::Driver(int ledPin, int dacRange, int stepSize, int interval)
    : _ledPin(ledPin), _dacRange(dacRange), _stepSize(stepSize), _interval(interval)
{
    _dutyCycle = 0;
    _previousMillis = 0;
}

int Driver::calibrate_bm(unsigned long currentMillis)
{
    if (currentMillis - _previousMillis >= _interval)
    {
        _previousMillis = currentMillis;

        _dutyCycle += _stepSize;
        if (_dutyCycle > _dacRange)
        {
            _dutyCycle = 0;
            analogWrite(_ledPin, _dutyCycle);
            delay(3000); // Wait after reset
        }
        analogWrite(_ledPin, _dutyCycle);
    }
    return _dutyCycle;
}