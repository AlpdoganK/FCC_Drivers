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

float Fuse_Weighted(float variance1, float variance2, float alt1, float alt2) {
    // Inverse-variance weighting
    float w1 = 1.0f / variance1;
    float w2 = 1.0f / variance2;

    return (alt1 * w1 + alt2 * w2) / (w1 + w2);
}

// A sensor stuck on a frozen value has near-zero residual against its own
// mean, so its raw variance collapses toward MIN_VARIANCE — which makes
// Fuse_Weighted trust it *more*, not less, for the whole confirmation window
// before stuck_count reaches BARO_STUCK_CONFIRM and it gets latched off
// outright. Inflate the variance fed to the blend in proportion to
// accumulated stuck evidence so its weight decays smoothly toward zero over
// that window instead of staying artificially high throughout.
static float Baro_EffectiveVariance(float variance, uint16_t stuck_count) {
    if (stuck_count == 0) return variance;

    float frac = (float)stuck_count / (float)BARO_STUCK_CONFIRM;
    if (frac > 1.0f) frac = 1.0f;

    return variance + (frac * frac) * BARO_STUCK_VARIANCE_INFLATION;
}

float Baro_FusedAltitude(BaroHealth_t *h,
                         SensorStats_t *s1, SensorStats_t *s2,
                         float alt1, float alt2)
{
    // Track exact repeats of the raw reading — a stuck sensor keeps
    // producing bit-identical output, independent of what its variance does.
    if (alt1 == h->last_alt1) {
        if (h->stuck_count1 < UINT16_MAX) h->stuck_count1++;
    } else {
        h->stuck_count1 = 0;
    }
    if (alt2 == h->last_alt2) {
        if (h->stuck_count2 < UINT16_MAX) h->stuck_count2++;
    } else {
        h->stuck_count2 = 0;
    }
    h->last_alt1 = alt1;
    h->last_alt2 = alt2;

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
            uint8_t s1_stuck = h->stuck_count1 >= BARO_STUCK_CONFIRM;
            uint8_t s2_stuck = h->stuck_count2 >= BARO_STUCK_CONFIRM;

            // A confirmed-frozen sensor is isolated outright — its low
            // variance is a symptom of being stuck, not evidence it's fine.
            if (s1_stuck && !s2_stuck) {
                h->sensor1_healthy = 0;
                return alt2;
            } else if (s2_stuck && !s1_stuck) {
                h->sensor2_healthy = 0;
                return alt1;
            }

            // Neither is confirmed frozen (or, rarely, both are): fall back
            // to isolating whichever has the worse variance history.
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
    float eff_var1 = Baro_EffectiveVariance(s1->variance, h->stuck_count1);
    float eff_var2 = Baro_EffectiveVariance(s2->variance, h->stuck_count2);
    return Fuse_Weighted(eff_var1, eff_var2, alt1, alt2);
}