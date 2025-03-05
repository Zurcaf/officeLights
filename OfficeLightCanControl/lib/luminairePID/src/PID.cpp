#include <pid.h>

// Constructor initializing PID parameters with member initialization list
pid::pid(float h, float K, float b, float alpha, 
         float Ti, float Td, float Tt, float N)
    : h{h}, K{K}, b{b}, Ti{Ti}, Td{Td}, Tt{Tt}, // Initialize sampling period, proportional gain, setpoint weight, 
                                                // integral time, and derivative time
      N{N}, I{0.0}, D{0.0}, y_old{0.0} {        // Initialize filter coefficient, integral term, derivative term, and previous output

    if (Ti == 0) I = 0;             // If integral time is zero, set integral term to zero
    if (Td == 0) D = 0;             // If derivative time is zero, set derivative term to zero
    if (Tt == 0) I_antiWind = 0;    // If derivative time is zero, set setpoint weight to zero

    I_gain = K * h / Ti;            // Integral gain
    I_antiWind = h/Tt;              // Anti-windup gain
    // Should check the validity of input arguments here (e.g., h > 0, K != 0, etc.)
}

// Destructor (empty as no dynamic memory is managed)
pid::~pid() {
}

void pid::update_pid(float _K, float _b, float _Ti, float _Td, float _Tt, float _NDer)
{
    K = _K;
    b = _b;
    Ti = _Ti;
    Td = _Td;
    Tt = _Tt;
    N = _NDer;
}

void pid::update_reference(float _r) {
    r = _r;
}

// Compute the control output (u) using PID formula
float pid::compute_control(float y) {

    float P = 0;

    if (alpha == 0)
    {
        P = b*r;
    }else
    {
        P = K * (b * r - y);            // Proportional term: weighted difference between reference and output
    }

    if (Td != 0)
    {
        float ad = Td / (Td + N * h);         // Derivative filter coefficient (a_d)
        float bd = Td * K * N / (Td + N * h); // Derivative gain coefficient (b_d)
        D = ad * D - bd * (y - y_old);        // Update derivative term using filter and difference of outputs
    }

    v = P + I + D;                      // Total control output: sum of proportional, integral, and derivative terms

    // Limit control output to range [0, 1] (e.g., for PWM or DAC constraints)
    if (v < 0) u = 0;
    if (v > 1) u = 1;

    return u;  // Return the computed control signal
}