/*
 * Filters for BME280 and MPU6050 sensor data (lpf, sma, complementary)
 * Author: Alpdogan
 * Created: 26 May 2026
 */

ifndef FILTERS_H
define FILTERS_H

include <stdint.h>
include <stdbool.h>

// Low-Pass Filter (LPF) for BME280 altitude data

typedef struct {
    float alpha;      // Smoothing factor (0.0 to 1.0). Lower = more filtering but slower response
    float prev_out;   // Previous filtered output value
    bool initialized; // Guard to prevent a massive lag spike on the first sample
} LowPassFilter_t;

void LPF_Init(LowPassFilter_t *lpf, float alpha);
float LPF_Update(LowPassFilter_t *lpf, float input);

// Simple Moving Average Filter (SMA) for BME280 altitude data

define SMA_WINDOW_SIZE 10  // Number of samples to average

typedef struct {
    float buffer[SMA_WINDOW_SIZE];
    uint16_t index;
    uint16_t count;
    float sum;
} SMAFilter_t;

void SMA_Init(SMAFilter_t *sma);
float SMA_Update(SMAFilter_t *sma, float input);

// Complementary Filter for MPU6050 angle estimation

typedef struct {
    float alpha;  /**< Weight given to Gyro data (typically 0.95 to 0.98) */
    float dt;     /**< Time step in seconds between samples (e.g., 0.01 for 100Hz) */
    float angle;  /**< The final filtered angle output (degrees) */
} ComplementaryFilter_t;

void Complementary_Init(ComplementaryFilter_t *cf, float alpha, float dt, float initial_angle);
float Complementary_Update(ComplementaryFilter_t *cf, float accel_angle, float gyro_rate);

endif /* FILTERS_H */