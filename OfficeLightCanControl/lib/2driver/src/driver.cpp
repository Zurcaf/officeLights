// luxmeter.cpp
#include <driver.h>


Driver::Driver(int ledPin, int dacRange, float stepSize, int interval)
    : _ledPin(ledPin), _dacRange(dacRange), _stepSize(stepSize), _interval(interval)
{
    _dutyCycle = 0;
    _previousMillis = 0;
    manualDutyMode = false; // Initialize manual mode to false
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

// Get duty cycle
float Driver::getDutyCycle()
{
    return _dutyCycle;
}

void Driver::setDutyCycle(float dutyCycle)
{
    if (manualDutyMode) // If in manual mode, do not change the duty cycle
    {
        return;
    }
        
    _dutyCycle = dutyCycle;
    int writedutyCycle = (int) (_dutyCycle * _dacRange);
    analogWrite(_ledPin, writedutyCycle);

    return;
}

void Driver::setManualMode(bool manualMode) {
    manualDutyMode = manualMode;
}

void Driver::setGainOffset(float _G, float _d)
{
    G = _G;
    d = _d;
}