#ifndef CAN_ROUTINES
#define CAN_ROUTINES

class canRoutines {
    public:
    // Constructor with default values for K, b, Ti, Td, and N
    explicit canRoutines(float h, float K = 1, float b = 1,
                 float Ti = 1, float Td = 0, float N = 10);

    // Destructor
    ~canRoutines();

};
#endif // DATA_STORAGE_METRICS