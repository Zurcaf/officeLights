#ifndef PCINTERFACE
#define PCINTERFACE

class pcInterface {
    public:
    // Constructor with default values for K, b, Ti, Td, and N
    explicit pcInterface(float h, float K = 1, float b = 1,
                 float Ti = 1, float Td = 0, float N = 10);

    // Destructor
    ~pcInterface();

};

#endif // pcInterface