#ifndef DATA_STORAGE_METRICS
#define DATA_STORAGE_METRICS

class dataStorageMetrics {
    public:
    // Constructor with default values for K, b, Ti, Td, and N
    explicit dataStorageMetrics(float h, float K = 1, float b = 1,
                 float Ti = 1, float Td = 0, float N = 10);

    // Destructor
    ~dataStorageMetrics();

};

#endif // DATA_STORAGE_METRICS