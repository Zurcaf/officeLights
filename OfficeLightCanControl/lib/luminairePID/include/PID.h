#ifndef PID_H
#define PID_H

class pid {
public:
    // Controller gains and parameters
    float I, D, K, Ti, Td, Tt, b, h, y_old, N;  // I: integral term, D: derivative term, K: proportional gain,
                                            // Ti: integral time, Td: derivative time, b: setpoint weight,
                                            // h: sampling period, y_old: previous output, N: derivative filter coefficient
    float r;                                // Reference value
    float u;                                // Control output
    float v;                                // Previous control output
    float alpha;                            // Setpoint weight

    float I_gain;                           // Integral gain
    float I_antiWind;                       // Anti-windup gain


public:
    // Constructor with default values for K, b, Ti, Td, and N
    explicit pid(float h, float K = 1, float b = 1, float alpha = 0,    
                 float Ti = 1, float Td = 0, float Tt = 1, float N = 10);



    // Destructor
    ~pid();

    // Update PID parameters
    void update_pid(float _K, float _b, float _Ti, float _Td, float Tt, float _NDer);

    // Update reference value
    void update_reference(float _r);

    // Compute control output based on reference (r) and measured output (y)
    float compute_control(float y);

    // Update internal state (housekeeping) for the PID controller
    void housekeep(float y);
};

// Inline implementation of housekeep to update integral term and store previous output
inline void pid::housekeep(float y) {
    float e = r - y;  // Error: difference between reference and measured output
    float e_s = u - v;

    I = I + I_gain * e * I_antiWind * e_s;  // Update integral term using proportional gain, sampling period, and integral time
    y_old = y;  // Store current output as previous output for next iteration
}

#endif // PID_H