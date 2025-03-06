#include <localController.h>

// Constructor initializing PID parameters with member initialization list
localController::localController(float h, float K, float b,
         float Ti, float Td, float N)
    : h{h}, K{K}, b{b}, Ti{Ti}, Td{Td},  // Initialize sampling period, proportional gain, setpoint weight, 
                                        // integral time, and derivative time
      N{N}, I{0.0}, D{0.0}, y_old{0.0} {  // Initialize filter coefficient, integral term, derivative term, and previous output
    // Should check the validity of input arguments here (e.g., h > 0, K != 0, etc.)
}

// Destructor (empty as no dynamic memory is managed)
localController::~localController() {
}

void localController::update_localController(float _K, float _b, float _Ti, float _Td, float _NDer)
{
    K = _K;
    b = _b;
    Ti = _Ti;
    Td = _Td;
    N = _NDer;
}

void localController::update_reference(float _r) {
    r = _r;
}

// Compute the control output (u) using PID formula
float localController::compute_control(float y) {
    float P = K * (b * r - y);  // Proportional term: weighted difference between reference and output
    float ad = Td / (Td + N * h);  // Derivative filter coefficient (a_d)
    float bd = Td * K * N / (Td + N * h);  // Derivative gain coefficient (b_d)
    D = ad * D - bd * (y - y_old);  // Update derivative term using filter and difference of outputs
    float u = P + I + D;  // Total control output: sum of proportional, integral, and derivative terms

    // Limit control output to range [0, 4095] (e.g., for PWM or DAC constraints)
    if (u < 0) u = 0;
    if (u > 4095) u = 4095;

    return u;  // Return the computed control signal
}