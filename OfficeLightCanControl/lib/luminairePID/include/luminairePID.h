#ifndef LUMINAIRE_PID_H
#define LUMINAIRE_PID_H

#include "pico/stdlib.h"

class LuminairePID {
private:
    // Controller gains in time-based form
    float K;      // Proportional gain
    float Ti;     // Integral time (seconds)
    float Td;     // Derivative time (seconds)
    float kf;     // Feedforward gain
    
    // Controller variables
    float setPoint;       // Desired illuminance (lux)
    float integral;       // Accumulated error
    float previousError;  // Error from last iteration
    float minOutput;      // Minimum output (0-100%)
    float maxOutput;      // Maximum output (100%)
    
    // Timing
    absolute_time_t lastUpdate;
    
    // Hardware parameters
    float maxIlluminance; // Maximum achievable illuminance (lux)
    
    // Helper function
    float clamp(float value, float min, float max);

public:
    // Constructor
    LuminairePID(float K_ = 0.1f, float Ti_ = 10.0f, float Td_ = 0.5f, 
                 float kf_ = 0.5f, float minOut = 0.0f, float maxOut = 100.0f,
                 float maxIllum = 1000.0f);
    
    // Set desired illuminance
    void setTargetIlluminance(float target);
    
    // Update controller with current illuminance
    float update(float currentIlluminance);
    
    // Get current setpoint
    float getSetPoint() const;
    
    // Set controller gains
    void setGains(float K_, float Ti_, float Td_, float kf_ = 0.5f);
};

#endif // LUMINAIRE_PID_H