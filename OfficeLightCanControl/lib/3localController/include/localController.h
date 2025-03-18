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
        _u, _v, _y,         // Desired output and output after saturation
        _error, _dutyError, // Error and duty error (u-v)
        _gain, _external,
        _k_x_b;             // Product of proportional gain and setpoint weight

    bool _integratorOnly; // Integrator only mode flag makes (u = rb + I)
    bool _bumpLess;      // BumpLess mode flag
    bool _occupancy;      // Occupancy control mode flag
    bool _feedback;        // Feedback control mode flag
    bool _antiWindup;      // Anti-windup control mode flag

    float lowerBoundOccupied;
    float lowerBoundUnoccupied;

public:
    localController(
        float h, float K, float b, float c,                                   // Sampling period, proportional gain, setpoint weight in proportional
        float Ti, float Td, float Tt, float N,                                // Integral time, derivative time, derivative filter coefficient
        bool integratorOnly, bool bumpLess, bool occupancy, bool feedback, bool antiWindup); // Integrator only mode flag, occupancy control mode flag, feedback control mode flag, anti-windup control mode flag
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

    // Get reference value
    bool getReference();

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
    
};

#endif // PID_H