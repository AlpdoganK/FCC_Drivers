typedef struct {
    FlightSM state;
    float alt_prev;
    float alt_peak;
    int8_t descent_count;
    float accel_threshold;
} ApogeeDetector;