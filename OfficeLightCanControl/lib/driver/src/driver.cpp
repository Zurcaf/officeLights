// luxmeter.cpp
#include <driver.h>

Driver::Driver(int ledPin, int dacRange, float stepSize, int interval)
    : _ledPin(ledPin), _dacRange(dacRange), _stepSize(stepSize), _interval(interval)
{
    _dutyCycle = 0;
    _previousMillis = 0;
}

float Driver::calibrate_bm(unsigned long currentMillis)
{
    if (currentMillis - _previousMillis >= _interval)
    {
        _previousMillis = currentMillis;

        _dutyCycle += _stepSize;
        if (_dutyCycle > 1.0f)
        {
            setDutyCycle(_dutyCycle);
            _dutyCycle = 0;
            return _dutyCycle;
        }
        setDutyCycle(_dutyCycle);
    }
    return _dutyCycle;
}

void Driver::setDutyCycle(float dutyCycle)
{
    _dutyCycle = dutyCycle;

    int writedutyCycle = _dutyCycle * _dacRange;
    analogWrite(_ledPin, writedutyCycle);
}