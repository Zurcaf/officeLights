#ifndef LOCAL_CONTROLLER
#define LOCAL_CONTROLLER

class localController {
public:
    // Controller gains and parameters
    float I, D, K, Ti, Td, b, h, y_old, N;  // I: integral term, D: derivative term, K: proportional gain,
                                            // Ti: integral time, Td: derivative time, b: setpoint weight,
                                            // h: sampling period, y_old: previous output, N: derivative filter coefficient
    float r;                                // Reference value

public:
    // Constructor with default values for K, b, Ti, Td, and N
    explicit localController(float h, float K = 1, float b = 1,
                 float Ti = 1, float Td = 0, float N = 10);

    // Destructor
    ~localController();

    // Update PID parameters
    void update_localController(float _K, float _b, float _Ti, float _Td, float _NDer);

    // Update reference value
    void update_reference(float _r);

    // Compute control output based on reference (r) and measured output (y)
    float compute_control(float y);

    // Update internal state (housekeeping) for the PID controller
    void housekeep(float y);
};

// Inline implementation of housekeep to update integral term and store previous output
inline void localController::housekeep(float y) {
    float e = r - y;            // Error: difference between reference and measured output
    I += K * h / Ti * e;        // Update integral term using proportional gain, sampling period, and integral time
    y_old = y;                  // Store current output as previous output for next iteration
}

#endif // PID_H