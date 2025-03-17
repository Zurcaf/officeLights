#include <localController.h>

// Constructor initializing PID parameters with member initialization list
localController::localController(
    float h  = 1, float K  = 0.1, float b = 1, float c = 0, // Sampling period, proportional gain, setpoint weight in proportional
    float Ti = 1, float Td = 0, float Tt = 10,
    bool officeLightsMode = false,
    float N = 10) // Integral time, derivative time, derivative filter coefficient
    : _h{h}, _K{K}, _b{b}, _c{c},
      _Ti{Ti}, _Td{Td}, _Tt{Tt},
      _officeLightsMode{officeLightsMode},
      _N_{N},
      _I{0.0}, _D{0.0}, _yOld{0.0},
      _r{0.0}, _y{0.0},
      _u{0.0}, _v{0.0},
      _error{0.0}, _dutyError{0.0},
      _bi{0.0}, _ad{0.0}, _bd{0.0}, _ao{0.0}
{
    // Prevent division by zero with safe fallbacks
    if (_Td + _N_ * _h <= 0) {
        // assert(false && "Td + N * h must be positive"); // Debug check
        _Td = 0.001; // Small positive default to avoid division by zero
    }
    if (_Ti <= 0) {
        // assert(false && "Ti must be positive"); // Debug check
        _Ti = 1.0; // Default to a reasonable integral time
    }
    if (_Tt <= 0) {
        // assert(false && "Tt must be positive"); // Debug check
        _Tt = 10.0; // Default anti-windup time
    }

    if (!officeLightsMode)
    {
        _ad = _Td / (_Td + _N_ * _h);            // Derivative filter coefficient (a_d)
        _bd = _Td * _K * _N_ / (_Td + _N_ * _h); // Derivative gain coefficient (b_d)
    }

    _bi = _K * _h / _Ti; // Integral gain coefficient (b_i)
    _ao = _h / _Tt;      // Anti-windup gain coefficient (a_o)
    _k_x_b = _K * _b;    // Product of proportional gain and setpoint weight
}

// Destructor (empty as no dynamic memory is managed)
localController::~localController()
{
}

void localController::update_localController(float K, float b, float c,
                                             float Ti, float Td, float Tt,
                                             float N)
{
    float new_k_x_b = K * b;

    _I += _error * (_k_x_b - new_k_x_b); // Update integral term using proportional gain, sampling period, and integral time
    _k_x_b = new_k_x_b;         // Store current output as previous output for next iteration

    _K = K;
    _b = b;
    _c = c;
    _Ti = Ti;
    _Td = Td;
    _N_ = N;

    if (!_officeLightsMode)
    {
        _ad = _Td / (_Td + _N_ * _h);            // Derivative filter coefficient (a_d)
        _bd = _Td * _K * _N_ / (_Td + _N_ * _h); // Derivative gain coefficient (b_d)
    }

    _ao = _h / _Tt;                          // Anti-windup gain coefficient (a_o)
}

void localController::update_reference(float r)
{
    _r = r;
}

// Compute the control output (u) using PID formula
float localController::compute_control()
{
    if (_officeLightsMode)
    {
        _v = _r * _b + _I; // Total control output: sum of proportional, integral, and derivative terms
    }
    else
    {
        float _P = _k_x_b *_error;               // Proportional term: product of proportional gain and error
        _D = _ad * _D - _bd * (_y - _yOld); // Update derivative term using filter and difference of outputs
        _v = _P + _I + _D;                      // Total control output: sum of proportional, integral, and derivative terms
    }

    _u = _v; // Control output is the desired output

    // Limit control output to range [0, 1]
    if (_v < 0)
        _u = 0;
    if (_v > 1)
        _u = 1;
    

    return _u; // Return the computed control signal
}

// Inline implementation of housekeep to update integral term and store previous output
void localController::housekeep(float y)
{
    _y = y;                                // Store current output
    _error = _r - y;                       // Error: difference between reference and measured output
    _dutyError = _u - _v;                  // Compute duty error (difference between desired and actual output)
    _I += _bi * _error + _ao * _dutyError; // Update integral term using proportional gain, sampling period, and integral time
    _yOld = y;                             // Store current output as previous output for next iteration
}