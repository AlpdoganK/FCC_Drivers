#include "flight_sm.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define DESCENT_CONFIRM 5 // Consecutive loops required to trigger apogee (~250ms)

// Global allocation visible outside this file
FlightState_t current_flight_state = FLIGHT_PAD;

// Latching Tablo 5 event flags — see flight_sm.h
FlightEvents_t flight_events = {0};

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

    // Clears every Tablo 5 bit. EK-7 2.1.4k allows the judges to run SUT more
    // than once with different flight profiles, so App_Run calls this when a
    // SUT starts and the next run begins from a blank status panel.
    flight_events = (FlightEvents_t){0};
}

void FlightSM_Update(float current_altitude, float ax, float ay, float az, float pitch_deg, float pitch_rate_gy) {

    switch (current_flight_state) {

        case FLIGHT_PAD:
            if (ax > 24.5f) {
                current_flight_state   = FLIGHT_BOOST;
                flight_events.liftoff  = true;  // KTE
            }
            break;

        case FLIGHT_BOOST:
            if (ax < 2.0f) {
                current_flight_state    = FLIGHT_COAST;
                flight_events.burn_time = true; // YSD
            }
            break;

        case FLIGHT_COAST:
            if (current_altitude > 500.0f) {
                current_flight_state       = FLIGHT_MIN_ALTITUDE_REACHED;
                flight_events.min_altitude = true; // IEA
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
            //
            // pitch_deg is the angle FROM VERTICAL: 0 = nose straight up,
            // 90 = horizontal, 180 = nose down. Both feeds honour that - the
            // IMU path adds the +90 offset in app.c, and the SUT test device
            // already reports angles that way.
            //
            // This test used to read `pitch_deg < 45.0f`, which was inverted
            // under both conventions and therefore ALWAYS TRUE: the IMU path
            // produced -90 while standing vertical on the pad, and the ground
            // software's angle sits near 0 at launch. A permanently true vote
            // collapsed the `baro && total >= 2` requirement below into "baro
            // alone", so a single barometric glitch sustained for
            // DESCENT_CONFIRM samples could fire the drogue with no
            // corroboration. Caught on the bench when GAA latched at 1041 m
            // while the rocket was still climbing.
            bool pitched_over  = (fabsf(pitch_deg) > 45.0f);
            bool rate_reversed = (pitch_rate_gy > 15.0f);
            v.orient_vote = (pitched_over || rate_reversed);

            // Tablo 5 wants the individual detections, not just the outcome:
            // bit 4 (ATE) is "altitude started falling" and bit 3 (GAA) is
            // "body angle / lateral acceleration over threshold". Both are
            // reported the moment the corresponding vote first goes true,
            // which is what makes the ground software's panel light up in
            // sequence rather than all at once when the drogue fires.
            if (v.baro_vote)   { flight_events.descent    = true; } // ATE
            if (v.orient_vote) { flight_events.body_angle = true; } // GAA

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
            Pyro1_Fire(); // Fire drogue (non-blocking, one-shot by pyro driver)
            flight_events.drogue_cmd = true; // SPE
            current_flight_state = FLIGHT_DESCENT;
            break;

        case FLIGHT_DESCENT:
            if (current_altitude < 800.0f) {
                current_flight_state = FLIGHT_MAIN;
                flight_events.alt_threshold = true; // BIT
                stability_timer = HAL_GetTick();
                last_alt_sample = current_altitude;
            }
            break;

        case FLIGHT_MAIN:
            // Fire main chute once on entry
            if (!main_chute_fired) {
                Pyro2_Fire();
                flight_events.main_cmd = true; // APE
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

uint16_t FlightSM_GetStatusBits(void) {
    uint16_t bits = 0;
    if (flight_events.liftoff)       { bits |= (1u << 0); } // KTE
    if (flight_events.burn_time)     { bits |= (1u << 1); } // YSD
    if (flight_events.min_altitude)  { bits |= (1u << 2); } // IEA
    if (flight_events.body_angle)    { bits |= (1u << 3); } // GAA
    if (flight_events.descent)       { bits |= (1u << 4); } // ATE
    if (flight_events.drogue_cmd)    { bits |= (1u << 5); } // SPE
    if (flight_events.alt_threshold) { bits |= (1u << 6); } // BIT
    if (flight_events.main_cmd)      { bits |= (1u << 7); } // APE
    return bits;                                           // bits 8-15 Reserve
}

uint8_t FlightSM_GetState(void) {
    switch (current_flight_state) {

        case FLIGHT_PAD:
            return 0;
            break;

        case FLIGHT_BOOST:
            return 1;
            break;

        case FLIGHT_COAST:
            return 2;
            break;

        case FLIGHT_MIN_ALTITUDE_REACHED:
            return 3;
            break;

        case FLIGHT_APOGEE:
            return 4;
            break;

        case FLIGHT_DESCENT:
            return 5;
            break;

        case FLIGHT_MAIN:
            return 6;
            break;

        case FLIGHT_LANDED:
            // System down. Recovery beacons active.
            return 7;
            break;
    }
    return 0xFF; // Invalid state (should never happen)
}