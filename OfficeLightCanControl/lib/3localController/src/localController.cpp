#include <localController.h>

// Constructor initializing PID parameters with member initialization list
localController::localController(
    float h, float Tk, float b, float c,                                                // Sampling period, proportional gain, setpoint weight in proportional
    float Ti, float Td, float Tt, float N,                                              // Integral time, derivative time, derivative filter coefficient
    bool integratorOnly, bool bumpLess, bool occupancy, bool feedback, bool antiWindup) // Integrator only mode flag, occupancy control mode flag, feedback control mode flag, anti-windup control mode flag
    : _h(h), _Tk{Tk}, _b{b}, _c{c},
      _Ti{Ti}, _Td{Td}, _Tt{Tt},
      _integratorOnly{integratorOnly}, _bumpLess{bumpLess}, _occupancy{occupancy},
      _feedback{feedback}, _antiWindup{antiWindup},
      _N_{N}, _gain{0.0}, _external{0.0}, _offset{0.0},
      _I{0.0}, _D{0.0}, _yOld{0.0},
      _r{0.0}, _y{0.0},
      _u{0.0}, _v{0.0},
      _error{0.0}, _dutyError{0.0},
      _bi{0.0}, _ad{0.0}, _bd{0.0}, _ao{0.0}
{
    constantCalc(); // Calculate the constants in the local controller
    if (_occupancy)
    {
        _r = _lowerBoundOccupied; // Set reference to lower bound if above threshold
    }
    else
    {
        _r = _lowerBoundUnoccupied; // Set reference to lower bound if below threshold
    }
}

// Destructor (empty as no dynamic memory is managed)
localController::~localController()
{
}

// Compute the control output (u) using PID formula
float localController::compute_control()
{
    float P = _Tk * (_b * _r - _y); // Proportional Term

    float ut = 0;
    float d = _y - (_gain * (_u * 4095.0f));
    float uff = (4095.0f / _gain) * _r - d;
    if (_feedback){
       ut = P + _I;
    }
    ut = ut + uff;

    float u_sat = ut; 

    // Saturation Limits
    if (ut < 0) {
        u_sat = 0;
    }
    if (ut > 4095) {
        u_sat = 4095;
    }

    // Anti-Windup with Back Calculation
    float K_aw = 1 / _Tt; // Anti-Windup Gain 
    _I += (_Tk * _h / _Ti) * (_r - _y) + K_aw * (u_sat - ut) * _h;


    //housekeep(r, y); // Update integral and previous measurement

    return u_sat/4095;
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

void localController::update_localController(float Tk, float b, float c,
                                             float Ti, float Td, float Tt, float N)
{
    float new_k_x_b = Tk * b;

    if (_bumpLess)
    {
        _I += _error * (_k_x_b - new_k_x_b); // Update integral term using proportional gain, sampling period, and integral time
    }

    _k_x_b = new_k_x_b; // Store current output as previous output for next iteration

    _Tk = Tk;
    _b = b;
    _c = c;
    _Ti = Ti;
    _Td = Td;
    _Tt = Tt;
    _N_ = N;

    constantCalc(); // Calculate the constants in the local controller  
}

void localController::constantCalc()
{
    // Prevent division by zero with safe fallbacks
    if (_Td + _N_ * _h <= 0)
    {
        Serial.println("Td + N * h must be positive. Setting Td to 0.001 to avoid division by zero.");
        _Td = 0.001; // Small positive default to avoid division by zero
    }
    if (_Ti <= 0)
    {
        Serial.println("Ti must be positive. Setting Ti to 1.0 as default.");
        _Ti = 1.0; // Default to a reasonable integral time
    }
    if (_Tt <= 0)
    {
        Serial.println("Tt must be positive. Setting Tt to 10.0 as default.");
        _Tt = 10.0; // Default anti-windup time
    }

    if (_occupancy)
    {
        _r = _lowerBoundOccupied; // Set reference to lower bound if above threshold
    }
    else
    {
        _r = _lowerBoundUnoccupied; // Set reference to lower bound if below threshold
    }

    if (!_integratorOnly)
    {
        _ad = _Td / (_Td + _N_ * _h);             // Derivative filter coefficient (a_d)
        _bd = _Td * _Tk * _N_ / (_Td + _N_ * _h); // Derivative gain coefficient (b_d)
    }

    _k_x_b = _Tk * _b;    // Product of proportional gain and setpoint weight
    _bi = _Tk * _h / _Ti; // Integral gain coefficient (b_i)

    _ao = _h / _Tt; // Anti-windup gain coefficient (a_o)
}

void localController::updateExternal()
{
    // Update the external illuminance
    _external = _y - (_gain * _u / 4096); // Calculate the external illuminance based on the measured output and control signal
    // Serial.printf("External: %.2f, Gain: %.2f, Duty Cycle: %.2f, LDR: %.2f\n", _external, _gain, _u, _y); // Debug output

}

void localController::setGainAndExternal(float gain, float offset)
{
    _gain = gain;         // Update the gain value
    _offset = offset; // Update the reference value

    constantCalc(); // Calculate the constants in the local controller
}

void localController::setReference(float r)
{
    _r = r;
}

void localController::setIntegratorOnly(bool integratorOnly)
{
    _integratorOnly = integratorOnly;
}

void localController::setBumpLess(bool bumpLess)
{
    _bumpLess = bumpLess;
}

void localController::setOccupancy(bool occupancy)
{
    _occupancy = occupancy;
    if (_occupancy)
    {
        _r = _lowerBoundOccupied; // Set reference to lower bound if above threshold
    }
    else
    {
        _r = _lowerBoundUnoccupied; // Set reference to lower bound if below threshold
    }
}

void localController::setFeedback(bool feedback)
{
    _feedback = feedback;
}

void localController::setAntiWindup(bool antiWindup)
{
    _antiWindup = antiWindup;
}

// Set lower bound for occupied state
void localController::setLowerBoundOccupied(float lowerBoundOccupied)
{
    _lowerBoundOccupied = lowerBoundOccupied; // Update the lower bound for occupied state
}

// Set lower bound for unoccupied state
void localController::setLowerBoundUnoccupied(float lowerBoundUnoccupied)
{
    _lowerBoundUnoccupied = lowerBoundUnoccupied; // Update the lower bound for unoccupied state
}

float localController::getExternal()
{
    updateExternal(); // Update the external illuminance value
    return _external; // Return the gain value
}

float localController::getReference()
{
    return _r;
}

bool localController::getIntegratorOnly()
{
    return _integratorOnly;
}

bool localController::getBumpLess()
{
    return _bumpLess;
}

bool localController::getOccupancy()
{
    return _occupancy;
}

bool localController::getFeedback()
{
    return _feedback;
}

bool localController::getAntiWindup()
{
    return _antiWindup;
}

float localController::getLowerBoundOccupied()
{
    return _lowerBoundOccupied;
}

float localController::getLowerBoundUnoccupied()
{
    return _lowerBoundUnoccupied;
}