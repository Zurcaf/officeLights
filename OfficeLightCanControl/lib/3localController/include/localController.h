#ifndef LOCAL_CONTROLLER
#define LOCAL_CONTROLLER

class localController
{
private:
    // Controller gains and parameters
    float _h, _K, _b, _c,   // Sampling period, proportional gain, setpoint weight in proportional, setpoint weight in derivative
        _Ti, _Td, _Tt,      // Integral gain, derivative gain, anti windup gain
        _N_,                // Derivative filter coefficient
        _I, _D, _yOld,      // Integral term, derivative term, previous output
        _r,                 // Reference value
        _bi, _ad, _bd, _ao, // Internal state variables
        _u, _v, _y,         // Desired output and output after saturation
        _error, _dutyError, // Error and duty error (u-v)
        _k_x_b;             // Product of proportional gain and setpoint weight

    bool _officeLightsMode; // Office lights mode flag makes (u = rb + I)
public:
    localController(
        float h, float K, float b , float c, // Sampling period, proportional gain, setpoint weight in proportional
        float Ti, float Td, float Tt,            // Integral time, derivative time, derivative filter coefficient
        bool officeLightsMode, float N);         // Anti-windup gain, derivative filter coefficient
    
    // Destructor
    ~localController();

    // Update PID parameters
    void update_localController(float K, float b, float c,
                                float Ti, float Td, float Tt,
                                float N);

    // Update reference value
    void update_reference(float r);

    // Compute control output based on reference (r) and measured output (y)
    float compute_control();

    // Update internal state (housekeeping) for the PID controller
    void housekeep(float y);
};

// Inline implementation of housekeep to update integral term and store previous output
inline void localController::housekeep(float y)
{
    _y = y;                                // Store current output
    _error = _r - y;                       // Error: difference between reference and measured output
    _dutyError = _u - _v;                  // Compute duty error (difference between desired and actual output)
    _I += _bi * _error + _ao * _dutyError; // Update integral term using proportional gain, sampling period, and integral time
    _yOld = y;                             // Store current output as previous output for next iteration
}

#endif // PID_H