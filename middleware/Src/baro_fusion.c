/*
 * Baro Fusion - Dual-Barometer Fusion & Fault Detection
 * Author: Alpdogan
 * Created: 24 May 2026
 */

#include "baro_fusion.h"

void Stats_Update(SensorStats_t *s, float x) {
    // Update mean with exponential smoothing
    float diff = x - s->mean;
    s->mean += s->mean_beta * diff;

    // Update variance with exponential smoothing
    float residual = x - s->mean; 
    s->variance = (1.0f - s->alpha) * s->variance + s->alpha * (residual * residual);
    
    if (s->variance < MIN_VARIANCE) {
        s->variance = MIN_VARIANCE;
    }
}

float Fuse_Weighted(SensorStats_t *s1, SensorStats_t *s2, float alt1, float alt2) {
    // Inverse-variance weighting
    float w1 = 1.0f / s1->variance;
    float w2 = 1.0f / s2->variance;
    
    return (alt1 * w1 + alt2 * w2) / (w1 + w2);
}

float Baro_FusedAltitude(BaroHealth_t *h,
                         SensorStats_t *s1, SensorStats_t *s2,
                         float alt1, float alt2)
{
    // Only update stats for sensors that are currently considered healthy.
    // If a sensor fails, its stats should freeze so it doesn't pollute its own history.
    if (h->sensor1_healthy) Stats_Update(s1, alt1);
    if (h->sensor2_healthy) Stats_Update(s2, alt2);

    // If a permanent fault has already isolated one sensor, bypass evaluation
    if (!h->sensor1_healthy) return alt2;
    if (!h->sensor2_healthy) return alt1;

    float disagreement = fabsf(alt1 - alt2);

    if (disagreement > BARO_DISAGREE_THRESH) {
        h->fault_count++;
        if (h->fault_count >= BARO_FAULT_CONFIRM) {
            // Isolate the faulty sensor based on worse variance history
            if (s1->variance > s2->variance) {
                h->sensor1_healthy = 0; // Latched off
                return alt2;
            } else {
                h->sensor2_healthy = 0; // Latched off
                return alt1;
            }
        }
    } else {
        // Slowly decay fault count if it was a transient glitch
        if (h->fault_count > 0) h->fault_count--;
    }

    // Normal operation: Both are healthy, return weighted fusion
    return Fuse_Weighted(s1, s2, alt1, alt2);
}