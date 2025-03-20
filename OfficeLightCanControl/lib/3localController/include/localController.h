#ifndef LOCAL_CONTROLLER
#define LOCAL_CONTROLLER

#include <Arduino.h>

class localController
{
private:
    // Controller gains and parameters
    float _h, _b, _c,   // Sampling period, proportional gain, setpoint weight in proportional, setpoint weight in derivative
        _Tk, _Ti, _Td, _Tt,      // general Gain, Integral gain, derivative gain, anti windup gain
        _N_,                // Derivative filter coefficient
        _I, _D, _yOld,      // Integral term, derivative term, previous output
        _r,                 // Reference value
        _bi, _ad, _bd, _ao, // Internal state variables
        _u, _manualDuty, _v, _y,         // Desired output and output after saturation
        _error, _dutyError, // Error and duty error (u-v)
        _gain, _external,
        _k_x_b;             // Product of proportional gain and setpoint weight

    bool _integratorOnly; // Integrator only mode flag makes (u = rb + I)
    bool _bumpLess;      // BumpLess mode flag
    bool _occupancy;      // Occupancy control mode flag
    bool _feedback;        // Feedback control mode flag
    bool _antiWindup;      // Anti-windup control mode flag
    
    float _lowerBoundUnoccupied = 1.0f; // Lower bound for unoccupied state
    float _lowerBoundOccupied = 5.0f; // Lower bound for occupied state

public:
    localController(
        float h = 0.01, float Tk = 1.0, float b = 1.0, float c = 0.0,                                   // Sampling period, proportional gain, setpoint weight in proportional
        float Ti = 1.0, float Td = 0.5, float Tt = 10.0, float N = 10.0,                                // Integral time, derivative time, derivative filter coefficient
        bool integratorOnly = false, bool bumpLess = true, bool occupancy = true, bool feedback = true, bool antiWindup = true); // Integrator only mode flag, occupancy control mode flag, feedback control mode flag, anti-windup control mode flag
    // Destructor
    ~localController();
    
    // Calculate the constants in the local controller
    void constantCalc();

    // Update PID parameters
    void update_localController(float K, float b, float c,
                                float Ti, float Td, float Tt, float N);

    // Compute control output based on reference (r) and measured output (y)
    float compute_control();

    // Update internal state (housekeeping) for the PID controller
    void housekeep(float y);

    // Set Duty manual
    void setDuty(float _manualDuty);

    // Update box Gain and external Illuminace
    void setGainAndExternal (float Gain, float External);

    // Update reference value
    void setReference(float r);

    // Set integrator only mode
    void setIntegratorOnly(bool integratorOnly);

    // Set BumpLess mode
    void setBumpLess(bool bumpLess);
    
    // Set occupancy control mode
    void setOccupancy(bool occupancy);

    // Set feedback control mode
    void setFeedback(bool feedback);

    // Set anti-windup control mode
    void setAntiWindup(bool antiWindup);

    // Set lower bound for occupied state
    void setLowerBoundOccupied(float lowerBoundOccupied);
    
    // Set lower bound for unoccupied state
    void setLowerBoundUnoccupied(float lowerBoundUnoccupied);

    // Get duty cycle
    float getDutyCycle();

    // Get reference value
    float getReference();

    // Get integrator only mode
    bool getIntegratorOnly();

    // Get bumpLess mode
    bool getBumpLess();

    // Get occupancy control mode
    bool getOccupancy();

    // Get feedback control mode
    bool getFeedback();

    // Get anti-windup control mode
    bool getAntiWindup();

    // Get lower bound for occupied state
    float getLowerBoundOccupied();

    // Get lower bound for unoccupied state
    float getLowerBoundUnoccupied(); 
    
};

#endif // PID_H