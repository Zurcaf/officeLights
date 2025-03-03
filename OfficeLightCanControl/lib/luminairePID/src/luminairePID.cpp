#include "LuminairePID.h"

// Constructor
LuminairePID::LuminairePID(float K_, float Ti_, float Td_, float kf_,
                          float minOut, float maxOut, float maxIllum)
    : K(K_), Ti(Ti_), Td(Td_), kf(kf_),
      setPoint(0.0f), integral(0.0f), previousError(0.0f),
      minOutput(minOut), maxOutput(maxOut), maxIlluminance(maxIllum) {
    lastUpdate = get_absolute_time();
}

// Clamp helper function
float LuminairePID::clamp(float value, float min, float max) {
    return (value < min) ? min : (value > max) ? max : value;
}

// Set target illuminance
void LuminairePID::setTargetIlluminance(float target) {
    setPoint = clamp(target, 0.0f, maxIlluminance);
    integral = 0.0f;
    previousError = 0.0f;
}

// Update controller
float LuminairePID::update(float currentIlluminance) {
    // Calculate time delta in seconds
    absolute_time_t now = get_absolute_time();
    float dt = absolute_time_diff_us(lastUpdate, now) / 1000000.0f;
    lastUpdate = now;
    
    // Calculate error
    float error = setPoint - currentIlluminance;
    
    // Feedforward term
    float feedforward = kf * setPoint / maxIlluminance;
    
    // Convert to PID coefficients
    float kp = K;
    float ki = (Ti > 0.0f) ? K / Ti : 0.0f;  // Prevent division by zero
    float kd = K * Td;
    
    // PID terms
    float proportional = kp * error;
    integral += ki * error * dt;
    float derivative = kd * (error - previousError) / dt;
    
    // Calculate total output
    float output = feedforward + proportional + integral + derivative;
    
    // Anti-windup and output limiting
    output = clamp(output, minOutput, maxOutput);
    if (output == minOutput || output == maxOutput) {
        integral -= ki * error * dt;
    }
    
    previousError = error;
    return output;
}

// Get current setpoint
float LuminairePID::getSetPoint() const {
    return setPoint;
}

// Set controller gains
void LuminairePID::setGains(float K_, float Ti_, float Td_, float kf_) {
    K = K_;
    Ti = Ti_;
    Td = Td_;
    kf = kf_;
}