/*
 * Baro Fusion - Dual-Barometer Fusion & Fault Detection
 * Author: Alpdogan
 * Created: 24 May 2026
 */

#ifndef BARO_FUSION_H
#define BARO_FUSION_H

#include <math.h>
#include <stdint.h>

typedef struct {
    float mean;
    float variance;
    float alpha;            // Variance tracking smoothing
    float mean_beta;        // Mean tracking smoothing
} SensorStats_t;

typedef struct {
    uint8_t sensor1_healthy;
    uint8_t sensor2_healthy;
    int16_t fault_count;

    // Frozen-value detection: a sensor stuck repeating its last reading
    // (e.g. a silently-failing I2C read) settles to a low self-variance just
    // like a real signal at a brief plateau, so variance can't tell the two
    // apart. Tracking exact repeats of the raw input catches it directly.
    float last_alt1;
    float last_alt2;
    uint16_t stuck_count1;
    uint16_t stuck_count2;
} BaroHealth_t;

#define BARO_DISAGREE_THRESH   25.0f    // meters
#define BARO_FAULT_CONFIRM     20       // consecutive samples before flagging
#define BARO_STUCK_CONFIRM     10       // consecutive identical raw samples before flagging frozen
#define MIN_VARIANCE           0.01f
#define BARO_STUCK_VARIANCE_INFLATION 1.0e6f // added to a sensor's variance, scaled by stuck-confirmation progress, to phase out its fusion weight before it's latched off

void Stats_Update(SensorStats_t *s, float x);
float Fuse_Weighted(float variance1, float variance2, float alt1, float alt2);
float Baro_FusedAltitude(BaroHealth_t *h, SensorStats_t *s1, SensorStats_t *s2, float alt1, float alt2);

#endif /* BARO_FUSION_H */