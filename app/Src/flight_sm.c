#include "flight_sm.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define DESCENT_CONFIRM 5 // Consecutive loops required to trigger apogee (~250ms)

// Global allocation visible outside this file
FlightState_t current_flight_state = FLIGHT_PAD;

// Private module variables
static ApogeeTracker_t apogee_tracker;
static uint32_t stability_timer  = 0;
static float    last_alt_sample  = 0.0f;
static bool     main_chute_fired = false; // One-shot flag for main chute

void FlightSM_Init(void) {
    current_flight_state         = FLIGHT_PAD;
    apogee_tracker.alt_peak      = 0.0f;
    apogee_tracker.descent_count = 0;
    stability_timer              = 0;
    last_alt_sample              = 0.0f;
    main_chute_fired             = false;
}

void FlightSM_Update(float current_altitude, float ax, float ay, float az, float pitch_deg, float pitch_rate_gy) {

    switch (current_flight_state) {

        case FLIGHT_PAD:
            if (ax > 24.5f) {
                current_flight_state = FLIGHT_BOOST;
            }
            break;

        case FLIGHT_BOOST:
            if (ax < 2.0f) {
                current_flight_state = FLIGHT_COAST;
            }
            break;

        case FLIGHT_COAST:
            if (current_altitude > 500.0f) {
                current_flight_state = FLIGHT_MIN_ALTITUDE_REACHED;
            }
            break;

        case FLIGHT_MIN_ALTITUDE_REACHED:
            if (current_altitude > apogee_tracker.alt_peak) {
                apogee_tracker.alt_peak = current_altitude;
            }

            ApogeeVotes_t v = {0};

            // VOTE 1: Barometric Descent
            v.baro_vote = (current_altitude < apogee_tracker.alt_peak - 1.5f);

            // VOTE 2: Kinematic Weightlessness
            float net_g = sqrtf(ax*ax + ay*ay + az*az);
            v.accel_vote = (net_g < 3.92f);

            // VOTE 3: Geometric Orientation Pitch-Over
            bool pitched_over  = (pitch_deg < 45.0f);
            bool rate_reversed = (pitch_rate_gy > 15.0f);
            v.orient_vote = (pitched_over || rate_reversed);

            uint8_t total_active_votes = v.baro_vote + v.accel_vote + v.orient_vote;

            if (v.baro_vote && total_active_votes >= 2) {
                apogee_tracker.descent_count++;
            } else {
                if (apogee_tracker.descent_count > 0) apogee_tracker.descent_count--;
            }

            if (apogee_tracker.descent_count >= DESCENT_CONFIRM) {
                current_flight_state = FLIGHT_APOGEE;
            }
            break;

        case FLIGHT_APOGEE:
            Pyro_Arm();
            Pyro_Fire(PYRO_CH1); // Fire drogue (non-blocking, one-shot by pyro driver)
            current_flight_state = FLIGHT_DESCENT;
            break;

        case FLIGHT_DESCENT:
            if (current_altitude < 150.0f) {
                current_flight_state = FLIGHT_MAIN;
                stability_timer = HAL_GetTick();
                last_alt_sample = current_altitude;
            }
            break;

        case FLIGHT_MAIN:
            // Fire main chute once on entry
            if (!main_chute_fired) {
                Pyro_Fire(PYRO_CH2);
                main_chute_fired = true;
            }

            // Reset stability timer if altitude is still changing
            if (fabsf(current_altitude - last_alt_sample) > 0.5f) {
                last_alt_sample = current_altitude;
                stability_timer = HAL_GetTick();
            }

            if (HAL_GetTick() - stability_timer > 3000) {
                current_flight_state = FLIGHT_LANDED;
            }
            break;

        case FLIGHT_LANDED:
            // System down. Recovery beacons active.
            break;
    }
}