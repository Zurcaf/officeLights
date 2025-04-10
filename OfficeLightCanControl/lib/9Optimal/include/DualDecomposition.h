#ifndef DUALDECOMPOSITION_H
#define DUALDECOMPOSITION_H

#include <Arduino.h>

// Define the Node class for the dual decomposition algorithm
class Node {
public:
    int id; // Node ID
    double c, k_own, k_other1, k_other2, L, d, q, alpha; // Parameters
    double u_ant, u_own, u_other1, u_other2, u_other3; // Primal variables
    double l_own, price_other1, price_other2, price_other3; // Dual variables and prices

    // Constructor
    Node(int id, double c, double k_own, double k_other1, double k_other2, 
         double L, double d, double q, double alpha);

    // Compute the u variable based on local and neighboring values
    double compute_u() {
        double u = (-c + l_own * k_own + price_other1 + price_other2) / (2 * q);
        u = 0.5 * u + 0.5 * u_ant; // Average with the previous value
        u = constrain(u, 0.0, 100.0); // Clamp the value between 0 and 100
        return u;
    }

    // Update lambda (dual variable) based on the current state
    double compute_l() {
        double l = l_own + alpha * (-u_own * k_own - u_other1 * k_other1 - u_other2 * k_other2 - d + L);
        return max(l, 0.0); // Ensure lambda is non-negative
    }

    // Compute the external price based on parameters and l_own
    double compute_price() {
        return l_own * (k_other1 + k_other2);
    }
};

#endif  // DUALDECOMPOSITION_H