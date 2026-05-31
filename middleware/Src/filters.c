/*
 * Filters for BME280 and MPU6050 sensor data (lpf, sma, complementary)
 * Author: Alpdogan
 * Created: 26 May 2026
 */

#include "filters.h"

// Low-Pass Filter Implementation

void LPF_Init(LowPassFilter_t *lpf, float alpha) {
    if (lpf == NULL) return;
    
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    
    lpf->alpha = alpha;
    lpf->prev_out = 0.0f;
    lpf->initialized = false;
}

/**
 * @brief Updates the LPF with a new raw measurement.
 */
float LPF_Update(LowPassFilter_t *lpf, float input) {
    if (lpf == NULL) return input;

    // Soft-start: If it's the first reading, seed it directly to prevent lag
    if (!lpf->initialized) {
        lpf->prev_out = input;
        lpf->initialized = true;
        return input;
    }

    // Formula: y[n] = α * x[n] + (1 - α) * y[n-1]
    float output = (lpf->alpha * input) + ((1.0f - lpf->alpha) * lpf->prev_out);
    lpf->prev_out = output;
    
    return output;
}

// Simple Moving Average Filter Implementation

void SMA_Init(SMAFilter_t *sma) {
    if (sma == NULL) return;
    
    sma->index = 0;
    sma->count = 0;
    sma->sum = 0.0f;
    
    for (uint16_t i = 0; i < SMA_WINDOW_SIZE; i++) {
        sma->buffer[i] = 0.0f;
    }
}

/**
 * @brief Updates SMA with a new sample using a rolling/sliding window buffer.
 */
float SMA_Update(SMAFilter_t *sma, float input) {
    if (sma == NULL) return input;

    // Subtract the oldest value we are about to overwrite from the running sum
    sma->sum -= sma->buffer[sma->index];
    
    // Add new value to buffer and running sum
    sma->buffer[sma->index] = input;
    sma->sum += input;
    
    // Advance circular buffer index
    sma->index = (sma->index + 1) % SMA_WINDOW_SIZE;
    
    // Track if buffer is full yet
    if (sma->count < SMA_WINDOW_SIZE) {
        sma->count++;
    }
    
    return sma->sum / (float)sma->count;
}

// Complementary Filter Implementation

void Complementary_Init(ComplementaryFilter_t *cf, float alpha, float dt, float initial_angle) {
    if (cf == NULL) return;
    
    cf->alpha = alpha;
    cf->dt = dt;
    cf->angle = initial_angle;
}

/**
 * @brief Fuses Accelerometer angle and Gyro angular rate to compute a stable angle.
 * @param accel_angle Angle calculated from accelerometers (stable short-term, noisy long-term)
 * @param gyro_rate Angular velocity from gyroscope in degrees/second (precise short-term, drifts long-term)
 */
float Complementary_Update(ComplementaryFilter_t *cf, float accel_angle, float gyro_rate) {
    if (cf == NULL) return accel_angle;

    // Angle = α * (Angle + Gyro_Rate * dt) + (1 - α) * Accel_Angle
    cf->angle = cf->alpha * (cf->angle + (gyro_rate * cf->dt)) + ((1.0f - cf->alpha) * accel_angle);
    
    return cf->angle;
}