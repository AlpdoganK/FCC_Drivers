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
} BaroHealth_t;

#define BARO_DISAGREE_THRESH   25.0f    // meters
#define BARO_FAULT_CONFIRM     20       // consecutive samples before flagging
#define MIN_VARIANCE           0.01f

void Stats_Update(SensorStats_t *s, float x);
float Fuse_Weighted(SensorStats_t *s1, SensorStats_t *s2, float alt1, float alt2);
float Baro_FusedAltitude(BaroHealth_t *h, SensorStats_t *s1, SensorStats_t *s2, float alt1, float alt2);

#endif /* BARO_FUSION_H */